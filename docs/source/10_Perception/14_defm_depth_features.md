# 冻结 DeFM 深度特征

`perception/models/defm/DefmEncoder` 消费实际的 `automsgs::sensor_msgs::Image`，
将米制深度交给包含官方预处理的冻结模型图，输出可共享的空间 patch tokens。
消息解码复用 `NativeDepthImage` 与 `DecodeDepthImage`，推理复用
`common::network::Engine`。代码没有另建消息协议或深度归一化算法。

## 模型与预处理契约

当前明确支持官方 `defm_vit_s14`。导出脚本为
`scripts/export_defm_depth_features.py`，需要显式提供官方源码、完整源码提交号、
实际预训练权重及其 SHA-256。权重严格加载，随后 `eval()` 并关闭参数梯度。
Git checkout 的提交和已跟踪内容必须匹配；源码 archive 的提交号由调用者
声明，清单记录实际 Python 源码哈希，不能将这个声明当作独立来源认证。

导出图先调用官方 `defm.utils.utils.preprocess_depth_batch`，再调用
backbone 的 `forward_features`。度量通道、无效值处理、官方 resize 与均值/
标准差都在图内。C++ 只按实际编码转换深度单位，不重写这些处理。
官方单图 NumPy 和批处理 Torch 路径的 float32 舍入差异独立核验；ONNX
始终与其实际导出的 Torch 批处理路径比较。

完整相机实例的输入为 `depth_metric [1,1,480,640]`，单位米、float32。
输出为 `normalized_depth [1,3,224,224]`、`patch_tokens [1,256,384]`、
`global_feature [1,384]`。模型加载检查名称、轴顺序、静态尺寸和元素类型，
只有元素总数相同不足以通过契约检查。目标尺寸需要是 14 的倍数。
导出采用 Dynamo / ONNX opset 18，保留官方 antialiased resize。

## 原生数据流与共享

1. 实际深度 Image 连同采集时间、光学坐标系及固定相机标定进入编码器。
2. 原生解码器处理 16UC1 / 32FC1、padding 和大小端；单位配置来自实际相机。
3. 模型图生成真实 tokens，并核验实际输出数量和有限性。
4. 同一份不可变 Image 的多个消费者获得相同的共享 const 结果。缓存默认
   保留四帧，后续控制周期只需编码新帧，不能把每周期预算机械计为四次推理。
5. 决策侧可读 patch/global 特征；适配过的策略通过
   `PolicyPipeline::SetDepthFeatures` 写入现有 RAW `TensorFrame` 和观测缓冲。
6. SanD 生成控制点后，样条求值、导数及起点约束仍委托给
   `common/math/bspline.hpp`；现有 NMPC / MPPI 接收 `automsgs::Path` 并输出
   `TwistStamped`。几何 Path 不携带定时轨迹的动力学保证。

特征结果保留原 Image、原采集时间、标定修订、原分辨率有效像素掩码和原
期限。无效或有效比例不足的帧不执行模型。缓存命中、等待锁和实际推理均
不得延长期限；默认最大观测年龄为 0.2 秒。CPU parity 测试显式使用 10 秒
期限，不能据此声称生产编码器满足 0.2 秒预算。

## 控制输入的严格适配

`MakeDefmTensorFrame` 只转发官方 patch/global 输出。策略侧须显式声明
mandatory RAW 输入及完全一致的静态形状，禁止补零、截断、轴交换及重复
归一化。现有 RAW 编码器也拒绝 payload/shape 不匹配与非有限特征。

有限采集期限写入现有进程内 `TensorFrame::valid_until_sec`，原时间戳保持
不变。期限在观测编码、策略推理后、原生跟踪后及安全过滤后检查。有限
期限过期时不会走缺失模态的零填充分支。`SetDepthFeatures` 要求已加载的
适配模型，拒绝配置为 ZERO_FILL 的观测策略。

直接使用 `controller.pipeline().SetDepthFeatures(*features, input_name,
"patch_tokens", now_sec, &error)` 可以消费编码器共享结果。应用应在自身
生命周期及标定协调内完成实际传感器订阅和供给。本接口没有偷偷安装
第二套相机订阅，也不意味着原 SanD 权重已适配 DeFM。

## 导出与验证

```bash
python3 scripts/export_defm_depth_features.py \
  --source /path/to/official/defm \
  --source-revision FULL_OFFICIAL_COMMIT \
  --weights /path/to/defm_vit_s14.pth \
  --weights-sha256 ACTUAL_CHECKPOINT_SHA256 \
  --output /path/to/new/defm_bundle
```

输出目录必须不存在。只在实际 ONNX Runtime 与 Torch parity 成功后原子
发布 `depth_features.onnx`、`parity.json` 和 `manifest.json`。fixture 来自
独立 Torch 推理，包含实际无效、负值及超量程输入，另核验常量与全无效
输入的模型输出；全无效帧在原生编码器入口仍被拒绝。

实际 native 检查使用 `AUTONOMY_DEFM_TEST_MODEL` 和
`AUTONOMY_DEFM_TEST_FIXTURE` 指向容器中的真实产物，测试名为
`autonomy.perception.defm_encoder`。缺少模型或 fixture 会失败，不能把跳过
计为通过。测试还覆盖原生字节布局、大小端、单位、缓存共享、错误形状、
原生 RAW 注入与过期零填充拒绝。

原生测试需要启用已有 `common/network/backend/onnx`。C++ SDK 根目录包含
`include/onnxruntime_cxx_api.h` 和 `lib/libonnxruntime.so`；仅安装 Python
wheel 不会提供这组 C++ 头文件。使用 `BUILD_ONNXRUNTIME=ON` 与
`OnnxRuntime_ROOT` 配置现有查找模块。测试注册会检查 `autonomy_common`
实际是否具有 `AUTONOMY_HAS_ONNXRUNTIME`，缺少后端时在 configure 阶段失败。

```bash
python3 scripts/validate_native_exploration.py \
  --manifest docs/validation/SOURCE_MANIFEST.json \
  --report docs/validation/NEW_VALIDATION.json \
  --defm-model /tmp/autonomy-defm-v2.47/depth_features.onnx \
  --defm-fixture /tmp/autonomy-defm-v2.47/parity.json \
  --defm-manifest docs/validation/2026-11-18-v2.47-defm-model-manifest.json \
  --onnxruntime-root /tmp/autonomy-onnxruntime-1.23.2-sdk \
  --artifact-wait-sec 0 --jobs 4
```

`SOURCE_MANIFEST.json` 是本次已同步源码清单，`NEW_VALIDATION.json` 必须
尚不存在。模型和 fixture 在测试前后校验摘要，报告保存逐步日志。
这里的 SDK 与模型位于容器临时目录，生产部署须配置持久安装位置。

具体运行记录及范围见 [原生检查](../17_Tasks/09_native_exploration_validation.md)。

## 尚需独立完成的验收

编码器与接口通过不证明 DeFM 特征已经训练成可导航的 SanD 条件，也不证明
SLAM、地图融合、地图保存或实机平滑性完成。四帧的时序融合、相机位姿
对齐及 DeFM-to-SanD 学习适配须由实际导出策略定义与验证。LiDAR-only
需要独立真实输入及适配模型，不能合成 RGB-D 来填补深度历史。

tokens 不提供实测自由空间、clearance、覆盖完成或定位健康证据。共享样条
的 C2 连续性不证明最终 Twist 的 jerk 有界；底盘命令与停止安全仍需要
执行侧核验。
