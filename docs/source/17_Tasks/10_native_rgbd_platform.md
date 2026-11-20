# RGB-D 探索平台的原生装配

`task_main --native_rgbd_config <yaml>` 将实际 Atlas 定位、RGB-D 观测、地图
融合与保存供应者装配到现有 ExplorationTask。TaskServer、Scheduler 和
BT 继续拥有任务及运动调度；平台通过它们已有的公开接口安装供应者。

实现位于 `autonomy/task/exploration/integration/native_rgbd_platform.hpp/.cpp`。
实际地图融合位于同目录的 `native_observation_mapper.hpp/.cpp`。
所有通信继续使用 automsgs，`autonomy/map` 源码及用户 Task/Planning
CMake 保持原有内容。根 CMake 的 NativeObservationMapping 接入文件
将任务适配层链接到已有 Atlas、ESDF 和 grid_map 实现。

## 初始化和数据流

1. 读取实际发布过的 CameraInfo 和 TFMessage Protobuf 文件，验证整流相机
   内参，并用现有 tf2 BufferCore 查询物理底盘到光学相机的安装链。
2. 订阅指定 Atlas Odometry 和 DiagnosticArray。NativeAtlasLocalization
   按原始整数采集时间关联健康状态和估计相机位姿，保存实际后端身份。
3. 在第一次实际健康位姿上，根据安装链建立以该时刻底盘为原点的导航坐标。
   底盘局部 Z 轴定义该导航坐标的上方向；地面分类仍由实际测量和安装高度
   完成。GT 不参与该初始化。导航底盘 TF 使用显式独立别名。
4. ConfigureNativeExploration 安装共享模式、相机标定、路由配置、真实
   localization lookup 和任务 owner 上的 packet processor。
5. NativeObservationSource 将深度、采集时刻定位和标定组成原子输入。
   processor 在实际消费时间再次验证输入，使用捕获的传感器代际及配置
   版本调用实际 mapper，随后向 Session 排入同一地图坐标的 map/packet。
6. mapper 复用 GroundClassifier、OccupancyGridMap、ESDF、Raycast 和
   grid_map；地图原生输出为 OccupancyGrid 与包含 elevation/ceiling
   等层的 GridMap。Snapshot 的高度未知值保持 NaN。
7. 启用 `host_vista_server` 时，同进程已有 PlannerServer 的 Vista 插件
   使用 mapper 的同一个不可变 MapReadView；实际导航与探索查询共享它
   的增量图和版本，不另外创建探索图。

## 融合与保存凭据

平台另发布原生 `measured_coverage_topic` 及其 DiagnosticArray 元数据。
数据只从 ExplorationSession 的 CoverageState 构造：-1 表示没有目标
账本要求，0 表示目标未观察，100 表示同一格子的全部要求（包括要求的
传感器）均由实际测量确认。中间值为按目标面积归一的测量质量。
这些数值不是障碍概率，也不是整屋面积覆盖率；不能交给占用地图插件。
元数据绑定 Session、地图世代、几何/覆盖版本及有效观测前缀。

覆盖快照不可变。新证据、要求失效或地图格网变化会建立新投影；已知
free 地图不产生测量质量，未观察的小面积要求不会因平均取整变成 100。
这一路输出为 GT 比较提供实际目标账本。完整房间覆盖还需独立空间域
要求及实际导航/观测，不能从 frontier 或 surface 目标的完成情况推导。

每个实际传感器包由任务代际、传感器种类和接收序号标识。generation 0
只产生启动几何，不能提供新 Session 的融合前缀。mapper 为接受的任务
代际建立独立 ledger；序号 3 已处理而序号 2 缺失时，前缀仍停在 1。
等待阶段重投同一输入不重复积分。

map refresh 返回实际映射结果及其连续完成前缀。无法跨过缺失序号，不能
用“队列空了”或发布计数构造同步结果。取消保留该请求的关闭记录，迟到
重投无法再得到完成结果。该适配层当前不会声称已经完成原始输入重放或
整域恢复；这些操作需要真正的数据及范围验证。

save 将实际 OccupancyGrid 和 GridMap 分别写入指定目录的
`<map_name>.occupancy.pb` 与 `<map_name>.grid.pb`，完成文件和目录 fsync
后才返回成功。缺失融合前缀、落盘失败或操作期限已过都会返回失败。
失败时可能保留已写入的部分文件，不能把文件存在当成任务保存成功。
这里保存的是导航/高度地图；SLAM 后端的因子图或 Atlas 状态保存是其
独立接口，不能由这两个文件推导。

MapReader 支持滚动窗口的历史保留策略与完整快照的未知替换策略。
mapper 使用完整快照策略：净空失效变为 unknown 时必须撤销旧 free。
同一未知 NaN 值不会反复增加几何版本。free 格子仍要求有效地面和净空。

## 配置

AutoSim 的渲染安装必须与发布 TF 一致。存在 URDF 相机安装时，验证
配置不应再用不一致的 `sensor_height` 覆盖渲染高度；v2.78 独立配置
去掉该覆盖，让渲染器和安装 TF 都使用实际 URDF 的 0.21 m 高度。
验证脚本直接检查实际 Habitat camera_local_offset 与 TF 高度，冲突
时拒绝启动定位验证。仅 CameraInfo 相同并不能证明外参正确。

GridMap 另保存 `navigation_state` 层：-1 为高度/可通行性未确认，0 为
实际保守导航 free，100 为 occupied。它与二维占用概率不同。已有
保存/恢复回归逐格检查该层与实际 MapReadView 的世界位置和状态一致；
该层用于追踪地图为何无法产生可执行视点，不能作为预测覆盖证据。

`ray_pixel_stride` 明确控制几何射线工作量。640×480 的完整深度有
307200 个像素；步长 4 只从对应实际像素构造几何射线。完整深度、无效
掩码及完整图像的有效比例仍保留给感知/控制，未采样像素没有被编码为
自由空间。该采样不能代替控制执行端对完整实时深度的安全验证。

下面的文件路径由实际运行记录生成；CameraInfo/TF 文件需要包含真实
发布数据。尺寸、余量、覆盖范围和 controller id 按平台实际配置填写。

```yaml
clock_domain: system
camera_info_file: native-camera-info.pb
tf_static_file: native-tf-static.pb
calibration_revision: 1
depth_topic: /validation/exploration/run/camera/depth/image_raw
camera_info_topic: /validation/exploration/run/camera/camera_info
minimum_depth: 0.1
maximum_depth: 8.0
ray_pixel_stride: 4
atlas_frame: atlas_optical_map
atlas_odometry_topic: /validation/exploration/run/atlas/odometry
atlas_status_topic: /validation/exploration/run/atlas/localization_status
physical_base_frame: base_footprint
navigation_frame: native_exploration_map
navigation_base_frame: native_exploration_base
base_odometry_topic: /validation/exploration/run/native/base_odometry
occupancy_topic: /validation/exploration/run/native/occupancy
grid_topic: /validation/exploration/run/native/grid_map
route_service: /get_plan
planner_id: vista_planner
controller_id: policy_controller
validated_sensor_mask: 2
host_vista_server: true
region: {min_x: -6.0, min_y: -6.0, max_x: 6.0, max_y: 6.0}
mapping_resolution: 0.15
robot_height: 0.25
robot_radius: 0.105  # AutoSim Turtlebot3 Burger circumscribed collision radius
vertical_margin: 0.03
horizontal_margin: 0.02
time_budget: 30.0
require_return: false  # enable only when the deployment requires a return leg
require_save: true
map_name: native_rgbd_exploration
map_output_directory: native-mapping
```

validated_sensor_mask 是明确提供的执行模式策略，不能据此声称真实运动
验证已经通过。该入口处理 RGB-D-only 的 Atlas 供应者；LiDAR 和双源仍
通过通用原生输入及模式接口接入各自实际定位器，不生成替代图像。

## 执行接入和验证状态

实际控制服务需要接收相同导航坐标的底盘位姿、速度、原始深度/DeFM
条件及独立执行许可。`controller_options_file` 可装配原有 ControllerServer，
通过原生 IO 读取实际校准反馈，并在独立地图校验后向指定通道写入
TwistStamped。没有该配置时不创建控制器，不替换所请求的插件。
控制配置与数据流详见 Control/controller 下的 `09_native_exploration_io`。
ControllerServer、SanD、已有 NMPC/MPPI 的实际输入与
安全状态机接入以及 DeFM 条件权重仍是完整移动探索验证的必要工作。
样条求值与导数继续复用 common/math 的公共 B 样条实现。

验证脚本的 `--task` 和 `--task-start` 使用实际 task 应用、原生 START
和反馈，并在运行时生成上述真实标定配置。即便启动和融合地图输出
通过，仍需要实际运动、覆盖账本、GT 比较及任务完成证据。
完整验收状态和原始记录集中在本目录的 native exploration 验证文档。

## 连续观测与航向执行

观测输入或覆盖账本的短暂丢失不会无条件取消已经准入的观测目标。当前
目标仍处于同一地图/传感世代，且原生平台用最新地图重新验证目标足迹后，
Session 保留观测阶段、导航请求和已经采集的 dwell 证据，只刷新完成判定
使用的版本号。目标真的落入障碍或已知不可通行区时才撤销导航并重选。

native Vista 的只读证据请求与路径请求共用一个客户端。前序请求尚未完成时，
新请求在自身超时预算内等待并回收旧 future，不再立即把并发查询当成探索
失败。仅用于返程时间估计的后台任务失败也不会撤销正在执行的安全观测目标。

启动观测支持累计 2π 原地旋转，发送给 `/spin` 的目标仍是相对转角，完成
校验使用轮询过程中累计的实测绝对转角以及最终航向误差。完成整周旋转后
机器人回到启动航向。探索视点分成两步：先用目标点方向作为路径终点航向，
使 SanD/MPPI 不必在路径起点完成不可跟踪的大角度掉头；到达位置后，再用
同一原生 `/spin` action 原地转向 `required_views` 指定的观察航向。位置到达
只校验平移容差，相机朝向由独立旋转结果确认。

路径编码器对任意数量的原生路径点按弧长均匀采样到模型要求的固定点数。
此前两点路径会被复制成前后两簇，而不是连续路径；现在模型输入、远端
MPPI 参考和公共 B 样条求值使用一致的几何顺序。

AutoSim 的 Turtlebot3 验证配置使用：

```yaml
startup_observation:
  enabled: true
  target_yaw: 6.283185307179586
  time_allowance: 10.0
  deadline_margin: 3.0
  maximum_position_drift: 0.08
  completion_tolerance: 0.08
  maximum_pose_age: 1.0
  support_radius: 0.8
  support_ring_radius: 1.2
  support_timeout: 60.0
  minimum_support_cells: 8
```

`target_yaw` 是该次 `/spin` 的相对转角。原生平台同时累计每次轮询的实测
航向增量，要求累计转角达到目标且最终航向回到期望角；仅检查首尾差值
无法证明整周扫描真的发生。真实部署应继续使用实测触发间隔、轮速上限
和底盘定位漂移配置，不能把 AutoSim 的数值直接当作硬件证明。
