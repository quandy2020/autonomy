# 05 AutoMsgs 集成

Atlas 的消息边界位于以下文件：

- `atlas/sensor/image_data.*`
- `atlas/sensor/imu_data.*`
- `atlas/sensor/pointcloud_data.*`
- `atlas/message_adapter.*`
- `atlas/atlas_node.*`

ImageData、ImuData 和 PointCloudData 是 Atlas 内部的强类型数据容器。
MessageAdapter 负责 AutoMsgs 消息与这些容器之间的转换；
AtlasNode 负责 Autolink reader、writer 与 MapBuilder 生命周期。

## 时间与坐标约定

- 图像和 IMU 转换必须保留消息时间戳，不使用到达时间替代采集时间。
- IMU 加速度和角速度单位分别为 `m/s²` 与 `rad/s`。
- 图像编码在进入 Tracker 前转换为配置要求的灰度或彩色布局。
- 位姿输出必须明确 body、camera 与 world 坐标系，不隐式交换外参方向。

## 所有权

适配层应尽量使用 `cv::Mat` 引用计数和只读视图，避免每帧深拷贝。任何
需要跨回调生命周期保存的数据都必须持有共享所有权或显式复制。Frame 和
KeyFrame 的描述子在构造后视为只读。

## 日志

节点和适配层统一使用 Autolink 的 `AINFO`、`AWARN` 和 `AERROR`。
不得新增 `std::cout`、`printf` 或 ORB-SLAM3 自定义日志宏。
