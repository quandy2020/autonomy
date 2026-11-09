# 16 · SpaceHero Linux 验证记录

验证日期：2026-11-09。执行环境：`quandy@192.168.66.124` 的 `SpaceHero` 容器，
workspace `/workspace/autonomy`，源码 `/workspace/autonomy/src/autonomy/autonomy/navigation`。

本文保留前一轮 73 项 CTest 与合成组件夹具的历史记录。后续真实 Habitat 多进程
验收、Safety 收敛修复与最新全量回归见 [18-autosim-validation.md](18-autosim-validation.md)。

## 构建与测试

使用用户指定的 `colcon build --packages-up-to autonomy`，五个组件均实际编译链接。
导航测试通过 `NAVIGATION_BUILD_TESTS=ON` 单独开启。最终日志与结果见本节末尾。
本轮清理临时调试对象后的完整构建耗时约 3 min 45 s；最后 Safety 修复后的增量
colcon 构建 4.62 s；补充深度重发运动断言后的最后构建 33.2 s。
最终配置为 Release (`-O3 -DNDEBUG`)、C++20。

```sh
cd /workspace/autonomy
colcon build --packages-up-to autonomy
ctest --test-dir build/autonomy/autonomy/navigation --output-on-failure --no-tests=error -j 8
```

首次启用 DeFM C++ 推理后端时使用：

```sh
colcon build --packages-up-to autonomy --cmake-args \
  -DNAVIGATION_BUILD_TESTS=ON -DBUILD_ONNXRUNTIME=ON \
  -DOnnxRuntime_ROOT=/workspace/autonomy/.deps/onnxruntime-linux-x64-1.23.2
```

仅设置 `BUILD_ONNXRUNTIME=ON` 不保证 SDK 已被找到。本轮补齐 CPU SDK 后实际运行
C++ Engine。随后普通 colcon 命令沿用已经确认的 CMake 缓存。

最终结果：**73/73 导航 CTest 通过，0 失败，19.29 s**。其中 72 项为核心/配置测试，
1 项为包含真实定时器、Action 和五组件的集成测试。
紧急停车后的加速度历史恢复回归已纳入 `safety_shield_implementation_test`。

安装位于 `/workspace/autonomy/install/autonomy`。逐文件 SHA-256 比较确认
conf 16 文件、DAG 3 文件、launch 4 文件、行为树 3 文件、docs 18 文件及 README
与源码一致，五个组件库存在，DAG 中的配置路径及 launch 的 DAG 引用均有效。
安装后的 DeFM 两份权重哈希也与本节制品记录一致；C++ 工具使用安装版 manifest
和配置根目录运行通过。

最终日志：`/tmp/navigation-final-colcon.log`、`/tmp/navigation-final-ctest.log`、
`/tmp/navigation-final-defm.log`、`/tmp/navigation-final-install.log`。
Shell 脚本 `bash -n`、导出脚本 Python 编译检查、主仓及 Autolink 子模块
`git diff --check` 均通过。文档更新后再次执行安装，保证安装树包含最终验收记录。

## 组件验收方法

`component_integration` 解析 navigation/safety/guardian 三 DAG，并直接实例化
ObservationBuilder、Navigation、Behavior、Safety、Guardian 五个组件。
测试运行于独立 IPC/挂载命名空间、独立 `/dev/shm` 和 `/tmp`，以免和现有
autosim 的共享内存及拓扑日志互相影响。Autolink 为 `MODE_REALITY`，真实定时器和
消息传输保持启用。传感器输入和差速运动由夹具生成，不包含真实碰撞物理模型。

服务启动时的历史消息接收任务会短暂占用任务协程，因此先等待异步调度就绪再验收。
点导航的 10 秒标准从新 Action 发起时开始计时，未把启动等待纳入。
最后一次夹具点导航 `(2,1,0)`：8.73841 s、位置误差 0.146964 m。

取消后的目标清理在 `/navigation/cmd_vel_raw` 检查零速；Safety 最终输出仍受
加速度/jerk 限幅，物理减速另行计算。不能把取消后某一时刻的非零最终速度直接
判定为缓存目标复活。Guardian DAG 的检查周期为 500 ms，失效测试覆盖两次检查
加 200 ms 接收租约；这不是 200 ms 内完成 Guardian 降级的性能证明。
探索夹具要求 2 s 内产生超过 0.02 m 的前进位移及正向原始命令；这一项不代表
Safety 在目标切换后的速度跟踪品质或 jerk 参数已经完成调优。
深度重发后也要求 2 s 内恢复超过 0.02 m 的前进位移，并检查最终正向速度。

## 本轮修复与回归

- Runner 串行控制入口；取消/抢占使正在推理或已经完成的旧结果失效，到达后输出零速。
- Action 与 topic 同步目标所有权；已见 topic 缓存独立记忆，取消后不会恢复旧 topic。
- 仿真策略使用 GoalSpec 位置容差，收敛到指定目标朝向，并在目标位于背后时优先转向。
- 深度租约只在 Header.stamp 变化时续租，以接收端 steady clock 计算 200 ms 新鲜度。
- 三模块心跳采用 steady clock；超时后允许新 stamp 的重启序号，拒绝缓存/乱序旧心跳。
- 组件 Clear 显式关闭辅助 reader；Node 的共享持有不会再保留同名订阅任务、阻止组件重建。
- Safety 平台短名从 `conf/platform/<name>.pb.txt` 解析。
- Safety 紧急零速清空可行性滤波的加速度历史；断流或近障停车后恢复不会继承停车前的减速方向。
  回归先建立减速历史，再分别触发断流/近障紧急停车，验证恢复正向速度且满足初始加速度限幅。
- Autolink 时间轮复制回调，取消标记在任务互斥锁下更新，避免裸回调指针悬空及停止后执行。
  回归覆盖任务队列被阻塞时停止定时器、周期定时器停止后不再执行。
- Autolink Action 取消响应携带真实目标 ID；回归验证序列化往返后数量和 ID 一致，
  以及复用响应对象时取消失败不会保留旧 ID。

## DeFM 制品与数值

制品存放在 navigation 的 `conf/models`，不提交权重到 Git。

- `defm_vit_s14.pth`：86,599,124 B；SHA-256
  `37a6e95befea3a16732a743b2ebec854fd5eaed912ebaf9fbffc63a2306f1e90`。
- `defm_vit_s14.onnx`：86,625,968 B；SHA-256
  `dafdb6557600a26e01e77c795262c0c880546765599ae2b62af5a52e34034dd2`。
- manifest `defm.pb.txt` 与 ONNX hash/IO 一致；许可证 `LICENSE.defm`。
- 输入 float32 `depth[1,3,224,224]`；输出 float32 `tokens[1,256,384]`、`pooled[1,384]`。
- zeros/random/ramp 三组 CPU C++ Engine 对 Python ORT 1.23.2：两个输出最大绝对误差均为 0。
- 独立 PyTorch/ORT 导出比较：tokens 最大绝对误差分别为 2.4557114e-5、1.3589859e-5、
  3.5154819e-4；pooled 分别为 2.8312206e-6、2.8610229e-6、1.8417835e-5。
  跨框架阈值为 5e-4，不能把这一组误差描述为全部小于 1e-4。
- C++ 校验覆盖 hash/名字/dtype/完整形状、错误输入长度/NaN，以及加载失败不保留旧模型。

导出环境 `/workspace/autonomy/.venvs/defm-export`；PyTorch 2.10.0+cu128 在 CPU 上执行，
ONNX opset 17，Python ORT 与 C++ SDK 均为 1.23.2。

```sh
NAVIGATION_CONF_DIR=/workspace/autonomy/src/autonomy/autonomy/navigation \
  /workspace/autonomy/build/autonomy/bin/navigation_defm_validator \
  /workspace/autonomy/src/autonomy/autonomy/navigation/conf/models/defm.pb.txt \
  /tmp/navigation-defm-reference
```

原始数值记录：`/tmp/navigation-defm-export.json`、`/tmp/navigation-defm-engine.log`。
参考输入和两个输出为 `/tmp/navigation-defm-reference/{zeros,random,ramp}.*.f32`。
导出/生成参考的完整命令见 [06-models.md](06-models.md)。

## 限制与后续

本轮不等于完整 autosim L1–L7：尚未验证 mainboard 动态加载、独立进程 kill/重启、
真实场景碰撞、长期运行、恢复次数反馈及完整行为树 XML 执行。
VNTPA planner 的真实权重、SR/SPL 和部署入口仍待接入。CUDA/TensorRT、ASan/TSan 和
目标硬件时延本轮未验收。构建仍存在 Autolink 头文件 warning，不声明全仓零 warning。

具体剩余项见 [13-todo.md](13-todo.md)。
