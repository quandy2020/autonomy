Air SLAM 重构设计总结
本文汇总了关于 Air SLAM 重构的完整讨论：目录结构、雷达兼容、LIO/VIO/LIVO 三模式、优化库选型、管理结构、容易遗漏的考虑点，以及嵌入式平台部署专项。

1. 总体设计目标
一套后端与地图，三种前端：VIO、LIO、LIVO 按配置装配，不是三套系统。

兼容雷达：传感器抽象统一，点云与图像走同一套同步、外参、时间对齐机制。

工程与算法分离：核心算法不依赖 ROS/具体硬件，通过接口通信。

配置驱动：同一可执行程序，换配置即换模式与平台。

嵌入式友好：算力、内存、实时性、功耗、可靠性均需纳入设计。

可测试、可降级、可观测：离线可跑，故障可切，运行可监控。

2. 完整目录结构
text
air_slam/
├── CMakeLists.txt
├── README.md
├── LICENSE
├── .clang-format
├── .gitignore
│
├── config/                              # 全局配置，不写死平台
│   ├── sensors/                         # 传感器标定与噪声
│   │   ├── camera/
│   │   ├── imu/
│   │   ├── lidar/
│   │   ├── gps/
│   │   ├── barometer/
│   │   └── optical_flow/
│   ├── frontend/
│   │   ├── vio.yaml
│   │   ├── lio.yaml
│   │   └── livo.yaml
│   ├── backend/
│   │   ├── iekf.yaml
│   │   └── graph.yaml
│   ├── map/
│   └── platforms/                       # 按机型+传感器组合
│       ├── drone_vision_only.yaml
│       ├── drone_lidar_imu.yaml
│       ├── drone_livo.yaml
│       └── vtol_livo_gps.yaml
│
├── docs/
│   ├── architecture.md
│   ├── coordinate_frames.md
│   ├── data_flow.md
│   ├── sensor_fusion_design.md
│   └── calibration_guide.md
│
├── include/air_slam/                    # 对外稳定 API
│   ├── common/
│   ├── sensor/
│   ├── frontend/
│   ├── backend/
│   ├── map/
│   ├── fusion/
│   └── pipeline/
│
├── src/
│   ├── common/                          # 基础设施，无业务依赖
│   │   ├── time/                        # 时间戳、时间对齐
│   │   ├── math/                        # SO3/SE3、插值、数值
│   │   ├── transform/                   # 外参树、坐标变换
│   │   ├── logging/
│   │   ├── config/                      # YAML 解析、参数校验
│   │   └── thread/                      # 线程池、无锁队列、内存池
│   │
│   ├── sensor/                          # 传感器抽象与适配
│   │   ├── interface/                   # SensorBase 统一接口
│   │   ├── camera/
│   │   ├── imu/
│   │   ├── lidar/
│   │   │   ├── driver/                  # Livox/Ouster/Velodyne
│   │   │   ├── preprocess/              # 去畸变、降采样、ROI
│   │   │   ├── deskew/                  # 运动补偿
│   │   │   └── filter/                  # 地面分割、离群点
│   │   ├── gps/
│   │   ├── barometer/
│   │   ├── optical_flow/
│   │   └── sync/                        # 硬触发/软同步、外参管理
│   │
│   ├── frontend/                        # 可插拔前端
│   │   ├── interface/
│   │   │   └── frontend_base.h          # 统一 OdometryResult
│   │   ├── factory/
│   │   │   └── frontend_factory.cpp     # 按配置创建前端
│   │   ├── vio/
│   │   │   ├── vio_frontend.cpp
│   │   │   ├── feature_tracker.cpp
│   │   │   ├── imu_preintegration.cpp
│   │   │   └── initialization.cpp
│   │   ├── lio/
│   │   │   ├── lio_frontend.cpp
│   │   │   ├── point_cloud_registration.cpp
│   │   │   ├── feature_extraction.cpp   # 面/线/边缘特征
│   │   │   ├── ikd_tree.cpp
│   │   │   └── map_manager.cpp
│   │   └── livo/
│   │       ├── livo_frontend.cpp
│   │       ├── loose_coupling.cpp       # 松耦合
│   │       ├── tight_coupling.cpp       # 紧耦合（可选）
│   │       └── visual_lidar_association.cpp
│   │
│   ├── backend/                         # 统一优化后端
│   │   ├── interface/
│   │   │   └── optimizer_base.h
│   │   ├── iekf/                        # IEKF 路线
│   │   │   ├── iekf_optimizer.cpp
│   │   │   └── state_update.cpp
│   │   ├── graph/                       # 因子图路线
│   │   │   ├── graph_optimizer.cpp
│   │   │   └── sliding_window.cpp
│   │   ├── factors/                     # 因子/残差，按传感器
│   │   │   ├── imu_factor.cpp
│   │   │   ├── visual_factor.cpp
│   │   │   ├── lidar_factor.cpp
│   │   │   ├── gps_factor.cpp
│   │   │   └── baro_factor.cpp
│   │   ├── marginalization/
│   │   └── loop_closure/
│   │       ├── visual_loop/             # DBoW/词袋
│   │       └── lidar_loop/              # Scan Context
│   │
│   ├── map/                             # 地图统一管理
│   │   ├── landmark/                    # VIO/LIVO 路标
│   │   ├── keyframe/
│   │   ├── elevation/                   # 空中场景高程图
│   │   ├── occupancy/
│   │   └── point_cloud/                 # LIO/LIVO 点云
│   │       ├── local_map/
│   │       ├── global_map/
│   │       ├── voxel_map/
│   │       └── map_merge/
│   │
│   ├── fusion/                          # 多源融合与降级
│   │   ├── ekf/
│   │   ├── factor_graph/
│   │   ├── outlier_rejection/
│   │   └── degradation/                 # 传感器失效检测与降级
│   │
│   ├── pipeline/                        # 系统编排
│   │   ├── slam_node/
│   │   ├── threading/                   # 前端/后端/回环线程
│   │   └── state_machine/               # 初始化/跟踪/丢失恢复
│   │
│   └── interface/                       # 对外接口层
│       ├── ros/
│       ├── grpc/
│       └── viz/
│
├── apps/                                # 可执行入口
│   ├── air_slam_node/
│   ├── offline_runner/                  # 离线数据集
│   ├── calibration_tool/
│   │   ├── camera_imu_calib/
│   │   ├── lidar_imu_calib/
│   │   ├── camera_lidar_calib/
│   │   └── joint_calib/                 # 三者联合
│   └── benchmark/
│
├── thirdparty/                          # 或改用包管理
│   ├── ceres/
│   ├── g2o/
│   ├── gtsam/
│   ├── sophus/
│   ├── opencv/
│   └── eigen/
│
├── test/
│   ├── unit/
│   ├── integration/
│   ├── dataset/
│   └── simulation/                      # Gazebo/AirSim
│
├── scripts/
│   ├── build.sh
│   ├── calibrate.sh
│   └── evaluate.py
│
└── tools/
    ├── trajectory_eval/                 # evo 等
    ├── map_visualizer/
    │   ├── pointcloud_viewer/
    │   └── image_viewer/
    └── log_analyzer/
3. 核心模块设计
3.1 传感器抽象
cpp
class SensorBase {
public:
  virtual bool init(const Config&) = 0;
  virtual bool start() = 0;
  virtual void stop() = 0;
  virtual TimeStamp latestTime() const = 0;
};
具体传感器继承并适配 ROS/驱动，核心算法只依赖接口。

3.2 前端统一接口
cpp
struct OdometryResult {
  TimeStamp t;
  SE3 pose;
  Vec3 velocity;
  Vec3 gyro_bias, accel_bias;
  Covariance cov;
  std::vector<Landmark> landmarks;   // VIO/LIVO
  PointCloud local_map;              // LIO/LIVO
};

class FrontendBase {
public:
  virtual bool init(const Config& cfg) = 0;
  virtual bool process(const SensorData& data) = 0;
  virtual bool getResult(OdometryResult& out) = 0;
  virtual void reset() = 0;
  virtual ~FrontendBase() = default;
};
工厂按配置创建：

cpp
std::unique_ptr<FrontendBase> createFrontend(const Config& cfg) {
  if (cfg.mode == "vio")  return std::make_unique<VioFrontend>(cfg);
  if (cfg.mode == "lio")  return std::make_unique<LioFrontend>(cfg);
  if (cfg.mode == "livo") return std::make_unique<LivoFrontend>(cfg);
  return nullptr;
}
3.3 后端统一
后端只认 OdometryResult 和因子接口，不关心前端类型。
可按配置选择 IEKF 或因子图，因子按传感器装配：

IMU 预积分因子

视觉重投影因子

雷达点到面/线因子

GPS/气压/光流先验因子

3.4 地图统一
视觉：路标点、关键帧地图

雷达：局部/全局点云、体素、八叉树

空中特有：高程图、占据栅格

回环后统一优化，地图融合时做一致性检查

3.5 融合与降级
松耦合：LIO 和 VIO 各自输出，再用 EKF/因子图融合

紧耦合：视觉重投影 + 雷达残差 + IMU 预积分统一优化

降级：LIVO → LIO / VIO → 纯 IMU 递推 + GPS/气压

4. 三种前端模式
模式	输入	前端核心	输出
VIO	图像 + IMU	特征跟踪 + IMU 预积分	位姿、速度、零偏、路标点
LIO	点云 + IMU	点云配准 + IMU 预积分	位姿、速度、零偏、局部点云地图
LIVO	图像 + 点云 + IMU	视觉特征 + 雷达点云 + IMU 联合	位姿、速度、零偏、路标点 + 点云地图
LIVO 融合方式：

松耦合：模块解耦，任一前端失效仍可降级运行；精度受各自前端上限限制。

紧耦合：精度高，视觉可给雷达提供纹理，雷达可给视觉提供深度；实现复杂，标定要求高。

建议：先松耦合跑通 LIVO，再紧耦合。

5. 优化库选择
方案	定位	适用场景
IEKF	紧耦合迭代扩展卡尔曼滤波	追求极致实时性、嵌入式部署，类似 FAST-LIO
Ceres Solver	应用最广、上手最快、自动微分方便	滑动窗口/因子图，视觉因子主导，快速原型
g2o	轻量、高度可定制	严格控制内存，自定义雷达残差边
GTSAM	理论最优、适合复杂回环	大规模全局一致性、多机协同、超长距离回环
建议：

若追求极致实时性与嵌入式部署：优先 IEKF（Eigen 实现流形迭代）。

若必须用图优化且注重开发效率：选 Ceres。

若要搞超大场景或协同建图：再看 GTSAM。

6. 配置驱动与平台切换
yaml
# config/platforms/drone_livo.yaml
frontend:
  mode: livo
  fusion: loose          # loose | tight
  vio:
    enabled: true
    camera: front
  lio:
    enabled: true
    lidar: mid360
  imu: pixhawk
sensor:
  gps: { enabled: false }
backend:
  type: iekf
同一套 air_slam_node，只换配置：

yaml
frontend: { mode: vio }
frontend: { mode: lio }
frontend: { mode: livo, fusion: loose }
7. 管理结构核心逻辑
7.1 三层分离
层	职责	是否依赖 ROS/硬件
common + sensor/interface	基础设施与抽象	否
frontend + backend + map + fusion	核心算法	否
interface + apps + pipeline	集成与运行	是
核心算法可脱离 ROS 在 offline_runner 中跑数据集。

7.2 前端可插拔，后端统一
frontend/interface/frontend_base.h 定义统一 OdometryResult

frontend/factory 按 config/platforms/*.yaml 创建 VIO/LIO/LIVO

backend 只认 OdometryResult 和因子接口

map 同时管理路标和点云，供三种模式复用

7.3 降级路径清晰
LIVO 视觉丢失 → 切 LIO

LIVO 雷达故障 → 切 VIO

VIO/LIO 都失效 → 纯 IMU 递推 + GPS/气压维持

7.4 标定与测试配套
calibration_tool/ 覆盖 camera-IMU、lidar-IMU、camera-lidar、联合标定

test/simulation/ 用 Gazebo/AirSim 生成带真值数据

tools/trajectory_eval/ 用 evo 评估 ATE/RPE

7.5 依赖与构建
CMake + vcpkg/conan 管理第三方，避免 thirdparty/ 臃肿

IEKF 路线只需 Eigen；因子图路线选 Ceres 或 GTSAM

ROS 适配只放在 src/interface/ros/，核心库不依赖 ROS

CI 跑 test/unit + test/integration，保证重构不破坏接口

8. 容易遗漏的考虑点
8.1 时间与同步
全局 TimeBase，所有传感器统一到同一时钟

硬同步 vs 软同步：相机与 IMU 尽量硬触发；雷达与 IMU 用 PPS + 时间戳

时间偏移在线标定：作为状态量加入后端优化

common/time/ 实现 TimeConverter，支持 PTP、GPS 周内秒、ROS Time 互转

8.2 标定体系
外参树：TransformTree，所有外参用 T_parent_child 表示，带协方差

在线外参标定：监控 T_imu_cam、T_imu_lidar 漂移

标定验证：重投影误差、点云重叠度自动打分

标定数据管理：带版本号、时间、温度，可回滚

8.3 坐标系与约定
命名规范：world、map、odom、body、imu、cam、lidar

重力方向与局部坐标系：world 定义为重力对齐的局部 ENU

统一 SI 单位、右手系，强类型防混用

8.4 初始化
VIO：视觉-IMU 对齐，估计尺度、重力、bias、速度

LIO：雷达-IMU 对齐，估计重力、bias

LIVO：联合初始化，视觉给尺度，雷达给结构

支持静止初始化 + 运动初始化，失败多次尝试

8.5 后端优化细节
可观测性：纯旋转/匀速下尺度不可观；长走廊雷达退化

一致性：FEJ 或 OC-EKF

边缘化：Schur 补，保持稀疏性和一致性

零偏与重力耦合：bias 随机游走建模要准

异常值剔除：视觉 RANSAC + 卡方；雷达点到面残差阈值 + 动态点剔除

8.6 地图管理
内存与生命周期：局部滑窗、全局降采样、LRU 淘汰

多分辨率：局部高分辨率配准，全局低分辨率回环/可视化

地图序列化：PCD、PLY、自定义二进制，带坐标系和版本

地图一致性：回环后统一优化，融合时一致性检查

高程图与占据栅格：更新频率和分辨率可配置

8.7 回环与全局一致性
视觉回环：DBoW3/DBoW2；雷达回环：Scan Context

回环验证：几何验证（PnP、ICP）+ 时序一致性

位姿图优化：回环后触发全局位姿图优化

多会话地图：支持加载历史地图做重定位

8.8 降级与故障恢复
传感器失效检测：图像过曝/欠曝/模糊、雷达稀疏/退化、IMU 饱和、GPS 跳变

降级策略：LIVO → LIO / VIO → IMU 递推

恢复策略：热插拔，重新初始化对应前端并融合

安全兜底：失控时输出最后有效位姿 + 协方差

8.9 实时性与线程模型
线程划分：传感器采集、前端、后端、回环、可视化

实时性保障：关键线程设优先级，避免日志/可视化阻塞

背压处理：队列满时丢旧帧或降采样

确定性：热路径避免动态内存分配，用内存池

8.10 空中平台特有挑战
振动：IMU 低通滤波、图像去模糊、雷达去畸变

大机动：雷达去畸变用 IMU 辅助，视觉用 IMU 预测搜索区域

GPS 拒止：纯 VIO/LIO 独立运行，GPS 恢复后平滑融合

高度估计：气压计 + 视觉 + 雷达融合

地磁干扰：不依赖磁力计，或做异常检测

温度漂移：IMU bias 温补模型

8.11 评估与基准
数据集：EuRoC、TUM-VI、NTU-VIRAL、M2DGR

指标：ATE、RPE、尺度误差、初始化时间、CPU/内存、回环成功率

回归测试：每次提交跑固定数据集，指标退化则阻断合并

仿真：Gazebo/AirSim 覆盖极端场景

8.12 日志与可观测性
结构化日志：分级、模块标签、异步写盘

运行时监控：帧率、残差、协方差、内存，支持 ROS topic 或 Prometheus

录制与回放：录制原始数据 + 中间结果

可视化：RViz、Pangolin、Web

8.13 配置管理
分层配置：默认 + 平台 + 运行时覆盖

参数校验：启动时校验范围和互斥性

配置版本：与代码版本对应

热更新：非关键参数运行时更新

8.14 依赖与构建
依赖管理：vcpkg/conan，锁定版本

可选依赖：雷达、GPS 作为可选编译项

交叉编译：提前支持 ARM 和 NEON 优化

构建时间：核心库与 ROS 适配分开编译

8.15 测试策略
单元测试：数学、变换、预积分

集成测试：离线数据集跑通 VIO/LIO/LIVO

模糊测试：配置解析、时间同步

性能测试：每模块耗时基准

硬件在环：条件允许时做 HIL

8.16 部署与嵌入式
算力预算：明确目标平台，分配 CPU/GPU/NPU

内存预算：点云地图、特征地图上限，超限降级

功耗：支持动态降频

启动时间：快速启动，地图异步加载

固件兼容：传感器固件版本矩阵

8.17 安全性
输入校验：传感器数据边界检查

异常处理：核心路径不抛异常，用错误码

看门狗：主线程加看门狗

数据安全：地图和日志加密存储

8.18 多机协同（若需要）
通信：DDS 或自定义协议

地图合并：分布式优化或中心化融合

时间同步：PTP

一致性：多机地图合并后检查

8.19 数据管理
数据集版本：带版本和校验和

数据标注：真值轨迹、标定参数、场景标签

数据隐私：脱敏或加密

8.20 文档与协作
架构文档：数据流图、模块依赖图

接口文档：Doxygen

决策记录：docs/adr/

新人上手：getting_started.md + 最小示例

9. 嵌入式部署专项
9.1 算力预算与异构计算
明确 CPU、GPU、NPU 分工：前端特征提取放 CPU，点云配准放 GPU，语义分割放 NPU

算力降级预案：负载过高时降低前端频率、减少特征点、跳过部分帧

NPU 推理延迟评估：如 RK3588 的 6 TOPS NPU 是否满足实时性

9.2 内存管理（嵌入式核心瓶颈）
点云量化：64 位 double 压缩到 8 位 char，DB-LIO 在 Jetson AGX Orin 上内存从 2888 MB 降至 524 MB

内存池与预分配：std::pmr 或自定义内存池，热路径避免动态分配

地图生命周期：四级边界架构，R-Tree 空间索引，LRU 淘汰 + 体素哈希

峰值内存控制：初始化、回环阶段峰值可能远超稳态，超限降采样

9.3 实时性与调度
实时内核：PREEMPT_RT + CPU 隔离 + IRQ 亲和性，延迟可从 2809µs 改善至 84µs

线程优先级：前端跟踪 SCHED_FIFO 高优先级，回环低优先级后台

CPU 核心绑定：前端绑高性能核，回环放能效核

确定性执行：热路径避免动态分配、系统调用、锁竞争

9.4 功耗与热管理
DVFS：根据负载动态调频调压，功耗可降 75%

热节流应对：监控温度，接近阈值主动降级

功耗预算：Jetson TX1 满载约 11W，空中平台电池有限

9.5 交叉编译与工具链
工具链一致性：锁定 GCC 版本，CI 验证目标平台链接

依赖库交叉编译：OpenCV、PCL、GTSAM 逐一交叉编译

ARM NEON 优化：SSE 替换为 NEON，或 sse2neon

9.6 可靠性与故障恢复
硬件看门狗：独立看门狗 + 窗口看门狗

故障检测与识别：FDI 系统，支持紧急降落或降级

进程级恢复：线程超时自动重启，主线程卡死复位

9.7 平台特有挑战
振动补偿：IMU 低通 + 图像去模糊 + 雷达去畸变

温度漂移：IMU 温补，外参在线估计并监控

算力-功耗-精度三角：不同飞行阶段不同优先级

9.8 部署与运维
启动时间：核心算法先启动，地图后台加载

OTA 更新：配置和地图远程更新，日志自动上传

资源监控：CPU/GPU/内存/温度/功耗，超限告警

10. 优先级与落地顺序
10.1 总体落地顺序
搭 common + sensor/interface + frontend/interface，定死 OdometryResult

实现 VIO，跑通 offline_runner

复用 IMU 预积分与后端，实现 LIO

加 LIVO，先松耦合再紧耦合

补 fusion/degradation 与 state_machine

最后接 ROS 与可视化

10.2 资源有限时的补强顺序
时间同步 + 外参树

统一接口 + 配置驱动

降级与故障恢复

地图内存管理

评估与回归测试

日志与可观测性

在线标定与时间偏移

多机协同（按需）

10.3 嵌入式专项优先级
内存池 + 点云量化

PREEMPT_RT + 线程优先级

硬件看门狗 + 进程恢复

DVFS + 热管理

交叉编译工具链

振动/温度补偿

算力降级策略

11. 结论
Air SLAM 重构的核心是：

统一抽象：传感器、前端、后端、地图四层接口稳定。

配置驱动：VIO / LIO / LIVO 是同一系统的三种前端配置。

嵌入式优先：内存、实时性、功耗、可靠性从设计之初就纳入。

可降级、可观测、可测试：空中平台安全底线。

按上述目录结构与管理逻辑推进，可在兼容雷达的同时，保持代码可维护、可扩展、可部署。

