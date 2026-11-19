# 冻结 DeFM 的 SanD 条件头训练

当前已部署的 SanD 权重输入为原始深度，不能通过改名或 reshape 接收
DeFM tokens。新增训练入口在 `scripts/training/train_defm_sand.py`，
条件编码器在同目录 `defm_sand_encoder.py`。它们用于训练真正的新条件
头。当前已在三个完整真实专家 run 上完成先导训练，并导出去噪 ONNX
图；完整专家集训练、原生控制循环消费和移动探索验收仍未完成。

## 代码与数据流

训练入口校验指定 Git 源码版本、DeFM checkpoint 和专家归一化统计
摘要，再严格加载官方 DeFM ViT-S/14。Backbone 全程冻结，训练模式也
强制它保持 eval，优化器只接收投影、条件融合及扩散头的可训练参数。
冻结参数及统计不允许通过反向传播或 train 切换改变。

输入保留原始度量深度 `B×4×1×480×640`，默认训练入口使用已经验证并
部署的冻结 DeFM ONNX 图获取 `B×4×256×384` patch tokens。该图包含
官方预处理和真实模型；入口核对它的导出 manifest、模型/源码和
checkpoint 摘要，避免在不同数值实现上训练后再混用端侧特征。
严格加载的 Torch backbone 仍保持冻结；本路径不重复执行它。
投影层对 token 空间网格做已有 Torch
池化并学习特征投影，加入时间/空间条件后输出 `B×384×256`，交给
上游条件 Transformer、目标编码器与条件 UNet/DDPM。它不是把旧
ResNet 特征权重改成另一名称。

`project_tokens` 接口也可直接消费已共享的真实 DeFM tokens，无需再次
调用 backbone，便于后续拆分感知与头部导出。这个接口目前只有局部
契约测试和先导 ONNX 张量一致性检查。去噪图输入为共享的四帧 tokens，
因此不会在条件头中包含第二个深度 backbone。

`FrozenOnnxDefmSandEncoder` 可缓存实际冻结图的输出。缓存位于该图
SHA256 的独立子目录，并按完整度量输入字节命名；它只缓存无时间属性
的模型特征，不续期观测，也不缓存定位/自由空间证明。时间嵌入由实际
历史顺序另行添加，异常形状、类型或非有限缓存拒绝使用。

## 专家监督与限制

复用指定 SanD 上游数据加载、专家标签、归一化与 v-prediction 损失。
保留 8 个控制点和首点锚定，不在这里实现任何新的样条算法。原生求值、
导数及执行继续使用 `common/math/bspline.hpp`。上游生成的专家控制点
是训练标签，不作为另一套原生执行数学实现。

数据加载器有在持续失败后返回 dummy 样本的路径。本入口显式拒绝该
路径，同时要求真实 run 身份、成功拟合、完整原始深度历史和实际归一化
目标。归一化失败不能回退到原始坐标训练，源深度单位由 `--depth-scale`
明确提供。训练/验证按上游 run 划分，记录实际损失与代码/权重来源。
入口现在要求 `--dataset-manifest`，逐文件检查字节数、SHA256、真实路径
以及数据集文件集合，并在保存 checkpoint 前重新校验。正式训练要求
完整 archive 摘要通过。明确启用 `--allow-pilot-dataset` 时，可以用已
逐成员校验的完整专家 run 调通训练链路；结果始终标记 pilot_only。
这个例外只用于先导验证，不满足完整训练或部署验收。

本阶段只使用实际四帧深度历史，**没有显式上一规划 token 条件**。
上游 initial-turn 标量与原生 previous_velocity 三维字段也不能视为
相同语义。下一阶段要提供因果的上一规划标签并训练/导出对应输入，
不能用当前专家未来控制点冒充上一规划。它是完整方案的剩余义务。

## 执行

先准备真正的专家数据、官方 DeFM 权重、它们的摘要，以及兼容上游的
Torch/diffusers/timm 等依赖。SanD 此次读取版本为
`53ada61e7477fdb045436402d4f40248ef9b336d`；DeFM 已验证感知导出版本为
`4849a368d669ae5d34bcd612fa3620f0e8a7ab83`。实际源码 checkout 必须与
指定版本一致，不能有未记录的 tracked 修改。

```sh
python3 scripts/training/train_defm_sand.py \
  --sand-source /path/to/sandplanner \
  --sand-revision 53ada61e7477fdb045436402d4f40248ef9b336d \
  --defm-source /path/to/DeFM \
  --defm-revision 4849a368d669ae5d34bcd612fa3620f0e8a7ab83 \
  --defm-weights /path/to/actual_defm_checkpoint.pth \
  --defm-weights-sha256 ACTUAL_CHECKPOINT_SHA256 \
  --defm-onnx /path/to/actual_depth_features.onnx \
  --defm-export-manifest /path/to/verified_defm_manifest.json \
  --token-cache /path/to/new_token_cache \
  --dataset /path/to/actual_experts \
  --dataset-manifest /path/to/verified_experts_manifest.json \
  --stats /path/to/actual_trajectory_stats.json \
  --stats-sha256 ACTUAL_STATS_SHA256 \
  --depth-scale 1000 \
  --output /path/to/new_training_run \
  --device cuda
```

示例路径及摘要需替换为实际输入，不能把示例字符串当作权重来源。
输出目录必须是新目录。训练只写本地记录，不上传到 wandb 或其他服务。
checkpoint 不重复保存共享 frozen backbone；重建时要重新严格加载相同
DeFM 权重。任何训练失败都会保留说明，不能据此得到部署通过状态。
checkpoint 的训练元数据保存为普通 JSON 类型，可直接使用
`torch.load(..., weights_only=True)`，不需要关闭安全加载模式。

## 去噪条件头导出

`scripts/training/export_defm_sand_head.py` 严格加载训练后的头部权重，
使用相同源码、冻结特征图、归一化统计和验证 run 进行静态 ONNX 导出。
图中保留条件 Transformer、UNet 和训练过的投影参数。自适应平均池化
按相同整数窗口展开，以支持 16→12 的输出网格；Torch 融合 Transformer
在导出阶段使用非融合路径，并先比较实际权重的两条路径。

最终输入为 `observation_depth_tokens [1,4,256,384]`、归一化目标
`observation_goal [1,3]`、控制点噪声 `sample [1,24]` 和原生扩散接口
使用的 float32 `timestep [1]`。输出 `action [1,24]` 为 v-prediction。
没有编码为新的底盘消息，也没有替代 common/math 的样条求值。

先导 checkpoint 必须显式使用 `--allow-pilot-checkpoint` 才能导出。
ONNX checker 和独立验证样本数值一致性通过后才发布新输出目录；
`native_runtime_consumption_verified`、`closed_loop_verified` 和
`deployment_ready` 仍为 false，避免把张量检查当成在线导航通过。

## 四帧原生输入与 C++ 推理

`MakeDefmHistoryTensorFrame` 将实际、不可变的 DeFM patch features 按
原始时间由旧到新组合为 `[1,T,N,D]`，复用现有 TensorFrame 和 RAW
编码接口。T 必须与模型静态轴一致（最多 8），N/D 和相机帧、尺寸、
标定版本必须相同。缺帧、重复/乱序、错误轴和任何过期输入均拒绝。
组帧只复制共享 tokens，不再次执行 DeFM，也不做第二次归一化。
输出保留最新采集时间与最早原始失效时间，不能用新帧续期旧上下文。

`--emit-native-fixture` 可从实际独立验证 run 导出原生 Image 文件和
Torch 参照输出。Image 的像素来自真实专家，时间和相机元数据明确
设为离线测试值；它们不证明真实定位健康或支持运动。夹具各文件和
实际头部/DeFM 权重都由摘要记录绑定，不能用合成图像或空权重替代。

v2.101 在 SpaceHero 中用当前真实 Autonomy 的 DeFM、TensorFrame/RAW
和 Engine 完成四组 C++ 推理校验，最大绝对误差约 `8.34465e-7`。
当另有原生服务运行且传输源仍在并行更新时，该检查在独立临时目录
采用现有生产目标的编译/链接参数，链接其真实已编译库。没有替代
头文件、库或模型，也没有重建正在运行的库；前后库及输入摘要一致。
这是实际 C++ 张量消费验证，不是在线 PolicyPipeline 或闭环控制结果。

全套固定端点的原生测试需要与其他运行实例隔离。v2.97 的特征/历史
检查通过，但全套中的路由服务与现存规划/控制程序竞争，记录保留
失败结果。验证脚本现在先检测这些实际进程，再拒绝启动未隔离的
全套检查；不能为了得到绿结果停止用户程序或忽略失败用例。

## PolicyPipeline 输入接入

`PolicyPipeline::SetDepthFeatureHistory` 将同一四帧输入装配到当前激活
模型声明的 RAW 槽。只有完整、未过期并与输入轴相符的共享特征可被
接受，ZERO_FILL 策略不适用于这一路。无效更新会替换对应槽为无效
输入，旧有效历史不会掩盖缺帧、标定错误或过期的新数据。调用者需要
序列化 Pipeline 的配置/生命周期和计算。

v2.106 在实际 SpaceHero 中使用真实 checkpoint 的去噪图，经过
Pipeline 加载配置、完整历史注入、已有 RAW 编码器与 PolicyModel
进行四组原生推理验证。对照同一专家/Torch 输出，检查通过；同时
验证了无效新历史使原有 RAW 不再可用，以及恢复有效历史的行为。
这一路复用现有模型与 TensorFrame，没有在头部再次调用 DeFM。

该测试直接调用模型推理，没有调用 ComputeCommand 或向底盘发送
速度。因此结果是 Pipeline 输入/模型消费验证，仍不是实时感知
回调、完整 SanD 采样/安全选择、NMPC/MPPI 闭环和停止确认的结果。
先导模型的部署标志保持 false；将这些接口用于实际控制还需要完成
对应验证以及剩余上一规划条件。

## 验证状态

本地八项张量契约测试覆盖冻结参数/统计、错误 token 输入、dummy/缺失
归一化拒绝、无效历史，以及共享 tokens 不重复执行 backbone。
测试使用明确的受控小模型与数据，**不证明新权重性能或导航闭环**。
另外已经使用原始 checkpoint 和一帧实际 RGB-D 记录验证相同冻结图的
直接输入/共享 token 投影路径，输出差异为零；它只验证单帧特征路径。
新 Torch 路径与旧端侧图在该帧上的小量数值偏差仍保留失败记录，没有
通过放宽阈值声明两者完全一致。

本次先导数据包括两个训练 run 和一个独立验证 run，总计 176 个真实
文件约 102.87 MB。三轮验证损失分别约为 0.3934、0.3247、0.2496。
四组验证样本的去噪 ONNX 输出与 Torch 误差小于 (10^{-6})。这些结果
证明训练和导出链路开始工作，不能证明泛化、安全或任务成功。

后续必须完成完整专家集训练、因果上一规划条件、原生控制输入消费及
数值检查、部署实时性，以及真实移动探索、返回、保存和
GT 覆盖对比。起点仍缺少安全初始化条件，不能由新增训练入口放行 unknown
包络。完整状态见 Tasks 下原生探索验证文档。
