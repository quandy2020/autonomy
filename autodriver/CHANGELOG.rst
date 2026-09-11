Changelog
=========

重大节点纪要（架构 / 真驱动 / 配置与构建契约）。不记日常重构。
包版本：``version.json`` → ``0.1.0``。

----

2026-09-11 — M6 双域与 Registry 定型
------------------------------------

**架构**

- 传感域：``SensorManager`` + ``SensorDriver``
- 本体域：``ChassisManager`` + ``ChassisDriver``
- 同进程、共享 Autolink ``Node``；底盘不链接 ``autonomy/vehicle``
- 本体消息：``vehicle_msgs.RobotState`` / ``RobotEvent``、``TwistStamped``、``Odometry``

**Registry**

- 统一 ``common/BackendRegistry`` + ``REGISTER_*_BACKEND``
- IMU / GPS 独立 Registry
- 串口 / CAN 基类：``SerialByteDriverBase``、``CanSensorDriverBase``

**3D 激光**

- 公共：``UdpScanDriverBase``、``point_cloud2_layout``、``beam_calibration_yaml``
- Velodyne / Hesai：UDP traits 分路径
- Livox：``assembler_driver_base``

**文档**

- 架构 / 数据流 / 术语（glossary）与双域、折叠相机、厂商手册对齐

----

2026-09-10 — M5 Chassis · Livox · RPLidar · 配置
------------------------------------------------

**Chassis**

- 顶层 ``chassis/``：``ChassisDriver``、``ChassisBackendRegistry``、``stub``（差分积分）
- ``main`` 启停 ``ChassisManager``

**Livox（3D）**

- 系统 SDK：Livox-SDK（v1）+ Livox-SDK2（非 vendored ROS driver）
- SDK2：Mid-360 / HAP / Mid360s / Avia2
- SDK1：Mid-40 / Mid-70 / Horizon / Avia / Tele
- 配置：``config/lidar/livox/*.yaml``
- CMake：``FindLivoxSDK`` / ``FindLivoxSDK2``；链接优先 shared（避无 -fPIC 静态库）

**RPLidar（2D）**

- 系统 ``rplidar_sdk`` + ``FindRplidarSDK``
- 安装 / udev：``scripts/install_rplidar_sdk.sh``、``create_udev_rules.sh`` → ``/dev/rplidar``
- 移除源码树 third_party 与相对路径探测

**配置**

- 相机折叠：``streams`` / ``point_clouds`` / ``imu``（loader 展开；扁平兼容）
- 厂商目录：``config/{camera,lidar}/<vendor>/``
- 校准迁入：``lidar/velodyne/``、``lidar/hesai/``（去掉顶层 ``params/``）

**进程 / 产物**

- ``main`` 逻辑收入 ``Run()``
- 产物约定：``build/autonomy/bin``、``build/autonomy/lib``

**文档**

- MkDocs：入门 / 设计 / 传感器 / API / FAQ
- 包根 ``README``：能力表与 CMake 开关

----

2026-09-06 … 09-07 — M4 相机与 UDP 激光
---------------------------------------

**相机**

- RealSense（librealsense2）hub 多流
- Orbbec（OrbbecSDK）hub 多流
- ``params_file`` + ``config/camera/<vendor>/``
- SmarterEye：Registry stub

**3D 激光（UDP）**

- Velodyne VLP-16；Hesai PandarXT / XT32
- 管线：PacketQueue → 方位角切帧 → Convert（``point_step=24``）
- 可选运动补偿 + ``PoseFeeder``（``compensator.pose_channel``）

**其它**

- canbus 骨架与 demo-circuit
- Radar / Microphone：Registry stub

----

2026-08 … 09-03 — M3 传感编排骨架
---------------------------------

**编排**

- YAML ``LoadConfig`` → ``SensorManager`` Attach / Detach
- ``SampleSink`` / ``bridge::Publisher`` → Autolink
- 可选 ``SensorHub`` 对齐旁路、udev 热插拔

**Module（内置 ClassLoader）**

- 采集：IMU、GPS、Camera、PointCloud、Lidar2d、Lidar3d
- 占位：Radar、Microphone、Range（attach-only）

**传输 / 协议**

- ``common::Stream``（串口 / UDP + 重连）
- NMEA、WitMotion parser；外参 ``LoadExtrinsicYaml``
- 诊断话题 ``/diagnostics``

**构建**

- ``BUILD_AUTODRIVER``
- ``AUTODRIVER_WITH_{REALSENSE,ORBBEC,…}``
- 嵌套 autonomy / colcon

----

2026-03-18 … 05 — M1 / M2 立项与类型
------------------------------------

- M1：引入 ``autodriver`` 包（统一传感器 HAL；后扩展为传感 + 本体）
- M2：配置类型与目录落地（``AutodriverConfig`` / ``version.json``）

----

Roadmap（意向，非承诺）
----------------------

- Chassis 厂商 SDK 真驱动（并行 / 替换 ``stub``）
- Radar Conti ProtocolData；Microphone PortAudio；SmarterEye SDK
- 其它 3D 激光 stub（rslidar / lslidar / …）按 Velodyne / Livox 模式实现
