# 探索任务的多传感器观测入口

观测入口位于 `autonomy/perception/observation/`，读取 Autolink 的原生
Image、CameraInfo 和 PointCloud2 消息，构造具有采集时刻、标定、位姿及
来源信息的 `ObservationPacket`。它不拥有定位器、地图或运动控制器。

## 三种传感器配置

- `lidar_only`：真实三维点云、该点云源的采集时刻定位证明，以及实际 LiDAR
  可视模型。没有 RGB-D 帧，不向模型填充虚拟图像。
- `rgbd_only`：真实深度帧、匹配的 CameraInfo、光学帧标定和采集时刻视觉定位证明。
  点云来自实际深度反投影，视场、深度范围和遮挡受相机模型约束。
- `lidar_rgbd`：两种观测分别保留来源、序号和质量，经过相同地图坐标世代的
  独立定位证明后交给消费者。输入顺序不能改变障碍物优先于自由空间的规则。

每种配置都需要能够独立工作的定位链路。代码不默认设备具有 IMU。
静态 TF、相同 frame 字符串或预填的初始位姿都不能代替定位健康证明。
运行时输入可用性与任务启动时的覆盖要求分别维护，传感器掉线不能悄悄
降低双源任务的验收要求。

## 数据流与身份

原生 Reader → 消息布局转换 → 单位/时间/标定校验 → 采集时刻定位查询
→ 原子观测包 → ExplorationTask 的身份化入口 → Session 的有限输入前缀
→ 实测覆盖评估及地图融合请求。

`sensor_message_decoder` 负责真实字节布局、步长、大小端、字段类型、有效
深度及点云读取预算。LiDAR 必须明确给出逐束时间策略；只有显式声明的
瞬时点云才按瞬时采集处理，不能默默把扫描中的运动忽略。

`ObservationIngress` 保持每条射线与采集时刻位姿证明的对应关系。
RGB-D 保留最近真实深度视图，历史缓存不意味着重建或刷新旧帧的采集时间。
`SensorDecodingOptions::rgbd_pixel_stride` 显式限制几何射线枚举。
默认 1 保留原有逐像素语义；原生 RGB-D 平台的 `ray_pixel_stride` 配置
用于控制建图工作量。完整深度视图和有效掩码始终保留，valid_fraction
仍按整个实际图像计算。只有选中且有效的像素产生实测射线；其余像素
没有变成无障碍射线。这一工作预算不替代完整深度的控制安全校验。
身份在解码前捕获，任务被替换后迟到的包不能进入新的任务世代。

## 定位处理延迟与有界关联

视觉定位器可能在深度 Reader 回调之后才完成同一帧的跟踪。原生源支持
显式设置 `localization_wait_timeout`，等待定位所有者提供**原始采集时间**
对应的健康证明。默认值为零，保留同步入口；平台需要延迟关联时应明确
配置等待时间，不能由输入源猜测时间域或定位健康。

启用后，Reader 只保留实际 automsgs 消息和收到深度时锁定的标定，并在
独立工作线程中重试。RGB-D 和 LiDAR 各有一个 FIFO，避免某一传感器等待
定位阻塞另一传感器。`localization_queue_size` 限制每个源的消息数量，
`localization_queue_bytes` 限制保留消息的序列化字节数；正在处理的队首
也计入这两个预算。默认分别为 8 条和 32 MiB。

`localization_retry_interval` 默认 5 ms；等待期限由 steady clock 计量，
测量有效期仍由原有传感器/定位时间域校验。等待不会更新帧时间、延长
有效期、制造缺失位姿或进行未获定位所有者确认的插值。lookup 应是有界、
不阻塞的查询；等待期限约束关联等待，不提供操作系统硬实时保证。
平台的 lookup 和 sink 必须允许工作线程调用。LiDAR 仍检查逐束原始时间，
不会因为末束位姿可用就省略其他射线的证明。

超时按原始接收序号报告丢失。队列满时拒绝新到消息，保留先前队首；
延迟结果记录自己的序号，不能读取处理结束时的最新接收计数。源快照
分别提供待关联数量、超时、溢出和关闭时释放数量，均不是地图融合凭据。
关闭先撤销输入并唤醒工作线程；ReaderRetirement 在独立清理线程中关闭
Reader、join 关联线程并释放保留消息。关联回调中的 `WaitForShutdown`
拒绝等待自己的线程，避免销毁源时自锁。

## Atlas 定位证明的原生缓存

`NativeAtlasLocalization` 接受现有 automsgs `DiagnosticArray` 和
`Odometry`，按原始 Header 的整数秒/纳秒关联。它提供
`ObservationIngress` 所需的采集时刻相机/底盘健康证明，不创建 SLAM、不
订阅 GT、不发布运动，也不拥有探索任务。平台可以从两个实际 Reader 回调
调用 `PushStatus` 和 `PushOdometry`，消息到达顺序不限。

只有同时间、同后端帧、同相机帧的健康状态和有效估计位姿形成有效记录。
诊断版本、实际 `rgbd` 模式、odometry endpoint、进程实例和原始地图身份
都要一致。原始 map id 和 coordinate epoch 可以为零；缓存维护自己的
非零、单调消费者 map epoch。进程重启或后端坐标变化撤销旧世界对齐，
迟到的旧身份状态不能恢复旧证明。

平台必须提供真实 `base_from_camera` 安装变换和非零标定版本，还必须通过
`SetWorldAlignment` 提供 navigation-from-backend 变换。对齐的 Header
必须锚定最新实际健康估计位姿；类本身没有默认世界位姿，也不从 GT 拟合
变换。`LatestBackendPose` 供平台执行真实初始化/安装链组合，
`WorldAlignment` 和 `map_epoch` 供地图读取者使用相同坐标身份。

`Lookup` 只返回指定采集时刻的证明，保持 RGB-D 定位支持掩码。它不会为
LiDAR 声称定位支持，不插值、不匹配最近时间、不以最新位姿替换旧测量。
有效期是采集时间加固定寿命；旧健康状态不会因后续查询续期。默认缓存
128 个完整或待关联记录，原生消息还受元数据大小限制。相同整数时间戳
包含矛盾状态/位姿时该记录失效；两个整数时间戳在 double 时钟中重合时，
该 double 查询被拒绝，不能猜测它对应哪一帧。

该缓存为真实平台装配提供定位 lookup。安装 Reader、相机标定验证、
世界初始化、实际建图融合和任务工厂仍由平台负责，缓存本身不证明探索
已经启动或覆盖已经完成。

## 丢失、降级与关闭

接收序号、最后处理序号、有效实测序号和地图融合序号是不同证据。
队列过载、过期、时间倒退以及位姿/质量拒绝都会留下丢失记录。
原生 Header 没有上游采集序号，因此本地回调序号只能审计接收入口，不能
证明传感器到 Reader 之间没有丢帧。

健康监视器独立检查年龄、有效比例、扫描完整度、定位支持与恢复持续时间。
掉线保持时间只稳定状态，不延长过期数据的有效期。
控制模式必须与已验证的输入源集合精确匹配。

`NativeObservationSource::Shutdown()` 先撤销输入许可，再请求释放 Reader。
`ReaderRetirement` 在回调线程外完成实际关闭与注销。
`WaitForShutdown()` 和关闭 future 用于区分已请求关闭与资源确实退役；
它们不证明机器人已经停稳。

## DeFM 的接入边界

DeFM 应消费经过官方度量预处理的真实深度图，冻结权重，并向决策/控制
提供共享 tokens。DeFM 特征不替代实测几何、定位证明或自由空间证据。
LiDAR 单源配置需要独立的 LiDAR 输入路径和训练/验证，不能伪造 RGB-D
历史来满足深度模型接口。

原生 `DefmEncoder` 已提供真实权重、官方预处理和现有推理后端的共享特征
入口，策略侧通过现有 RAW TensorFrame 进行严格形状和采集期限校验。
实现、导出契约和验收边界见 [冻结深度特征](14_defm_depth_features.md)。
目标设备耗时、DeFM-to-SanD 训练适配和三个控制模式的模型验收仍需分别
完成；观测入口测试通过不能作为这些模型已经部署的证据。

任务侧的丢失恢复见 [多传感器探索任务](../17_Tasks/08_multisensor_exploration.md)，
验证命令见 [原生检查](../17_Tasks/09_native_exploration_validation.md)。

## 任务世代与实际订阅的连接

`ExplorationTask::ConfigureObservationSource()` 接收 TaskServer 实际持有的
shared task、明确的源配置和真实定位器的采集时刻查询。配置必须与 Session
的 `SensorMode` 完全一致，并在任务空闲时安装；输入回调只持有 weak task。
应用可通过 `ConfigureNativeExploration()` 一次安装原生 Session 工厂和源，
详见 [任务组装](../17_Tasks/08_multisensor_exploration.md)。平台仍须提供真实
地图输入与定位器，调用接口不等于这些后端已经配置完成。

原生工厂将实际覆盖相机写入 `expected_depth_camera`，内参及深度有效范围
在安装前检查；接收到的 CameraInfo 必须匹配相同的 width/height/fx/fy/cx/cy。
不一致会清空标定并锁存错误。`depth_calibration_ready` 仅在实际匹配
CameraInfo 被处理后为真，Reader 创建成功不能提前标记相机可用。
采集时刻定位还必须与覆盖外参、坐标系及标定修订一致。独立接入旧接口时
该预期相机参数可不设置，此时仍按原有修订规则锁存实际首份标定。

生产调用位于 `ExplorationTask::Impl::Run()`：任务 owner 每轮先处理生命周期
命令，再协调实际 `NativeObservationSource`。首次 START 前建立世代零输入，
只允许定位就绪种子。START 接受后，旧回调立即失去新世代准入权；owner
请求旧 Reader 退役，等真实关闭 future 成功，再建立最新 START 世代的订阅。
连续 START 只为最新世代重建，不逐个重放已经被替换的请求。

暂停/恢复保留相同 Reader 和接收序号，由 Session 排除暂停期间的覆盖证据。
取消、完成或失败后停止输入，后续 START 建立独立的新世代订阅。配置、Node
或共享时钟更新也需要旧 Reader 成功退役，不能在相同 channel 上覆盖注册。

退役等待使用 `steady_clock`，独立于任务暂停与传感器时间。owner 轮询完成
future，不阻塞等待原生回调退出。超时、关闭失败、Reader 创建失败和未经
新标定版本确认的内参变化会锁存输入错误，阻止自动重建和新 START。
超时后实际退役若最终成功，仍保留此前错误，需外部诊断后重建输入/任务。
`GetObservationSourceSnapshot()` 提供原因与真实 Reader 状态；task Shutdown
并 join owner 后，`ObservationSourceShutdownCompletion()` 用于核验最终
输入清理。它们均不证明机器人停稳、地图融合完成或 SLAM 健康。

原生消息转换、Source 和 `ReaderRetirement` 统一由
`autonomy_navigation_support` 编译一次，Task 与可选 Perception 域共享该实现。
原生输入测试链接同一个生产目标，不再在测试里单独重编这些实现。
