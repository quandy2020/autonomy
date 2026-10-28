# 02 架构

Atlas 按职责划分模块，公开头文件根目录为项目根目录。

- `atlas/common/`：时间、线程池、阻塞队列、配置及通用基础设施。
- `atlas/common/io/`：配置、轨迹和地图序列化。
- `atlas/common/math/`：几何计算、矩阵转换和两视图重建。
- `atlas/common/optimization/`：g2o 顶点、边和优化流程。
- `atlas/sensor/`：Frame、KeyFrame、MapPoint、IMU 数据与相机模型。
- `atlas/feature/`：ORB 特征提取与 FBow 词袋封装。
- `atlas/match/`：ORB、PnP 和 Sim3 匹配。
- `atlas/tracking/`：前端跟踪、初始化和重定位。
- `atlas/mapping/`：局部建图与局部 BA。
- `atlas/closure/`：地点识别、回环检测和位姿图校正。
- `atlas/map/`：单地图和多地图生命周期。
- `atlas/message_adapter.*`：AutoMsgs 与内部类型转换。
- `atlas/atlas_node.*`：Autolink 节点集成。

## 数据流

图像与 IMU 首先由消息适配层转换为内部数据，再交给 Tracker。Tracker
创建 Frame、维护当前位姿并查询局部地图；LocalMapper 消费关键帧并更新
MapPoint；LoopCloser 使用 FBow 查找候选并调用位姿图优化。Atlas/Map
持有最终地图状态。

## 依赖方向

`common` 提供跨模块基础能力。sensor、feature 和 match 构成数据与前端
算法层；tracking、mapping 和 closure 依赖这些底层模块。map 负责所有权，
System/MapBuilder 只负责编排。

`common/optimization` 封装 g2o 细节。其他模块只能通过该目录公开的
Optimizer 和自定义图类型访问优化后端，避免在业务模块重复构建求解器。

## 并发模型

Tracking 在调用线程运行，LocalMapping 与 LoopClosing 使用独立后台线程。
双目左右 ORB 提取在启用 OpenMP 时使用固定两个 section；Eigen 的内部
并行被关闭，防止与 OpenCV、g2o 和后台线程形成过度订阅。
