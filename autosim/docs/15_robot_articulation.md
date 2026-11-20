# Habitat 中的机器人模型与自由度控制

机器人资源位于 `robot/<profile>/`，每个目录包含 URDF、引用的展示 mesh、来源记录 `source.json` 和上游许可证（如有）。Google 使用保留轮子连续关节的 ManiSkill URDF，因此可以控制轮子、机械臂、夹爪和头部。宇树选用 G1 人形机器人（29 自由度）和 Go2 四足机器人（12 自由度）。轮式平台选用 TurtleBot3 Burger（2 个轮子关节）和 Clearpath Husky（4 个轮子关节）。五个模型各自独立存放。

| 相关 | 链接 |
|------|------|
| 详细使用 | [16](16_usage.md) |
| 配置 | [05](05_configuration.md) |

## 查看与操作

在已有 SpaceHero 容器中运行：

```bash
docker exec -it \
  -w /workspace/autonomy/src/autonomy/autosim \
  SpaceHero scripts/run_robot_viewer.sh \
  --profile google_robot --host 0.0.0.0 --port 8765
```

浏览器访问 `http://192.168.66.124:8765`。顶部型号选择可在同一 Habitat 场景中切换机器人。查看器也订阅 `/cmd_vel`，页面上按住前进/后退/左转/右转按钮即可控制；松开停止。深灰背景便于看清黑色小腿和脚部。右侧为所有关节的滑块和数值输入，支持重置姿态、调整观察方位、仰角和距离。图像由 Habitat-Sim 渲染，浏览器只是显示画面和传送控制命令。默认监听 `127.0.0.1`；上述 LAN 示例依赖容器使用 host 网络。

使用 `--profile unitree_g1` 查看人形机器人，使用 `--profile unitree_go2` 查看四足机器人。`--scene /path/to/scene.glb` 可加载已有 Habitat 场景。

无需启动 Habitat 即可查看关节清单：

```bash
python -m autosim.robot.viewer --profile google_robot --list
```

导出一张姿态图：

```bash
python -m autosim.robot.viewer --profile google_robot \
  --joint joint_elbow=0.6 --joint joint_head_pan=0.3 \
  --output /tmp/google_robot.png
```

## 接入 autosim bridge

为每个模型提供 `config/robots/<profile>.yaml`。启动仿真和键盘控制（w/x 前后，a/d 转向，空格/s 停止）：

```bash
SCENE=empty ROBOT=unitree_go2 SENSORS=basic scripts/run.sh both
```

也可以只启动仿真：`scripts/run.sh sim --scene empty --robot unitree_go2 --sensors basic`。

控制频道 `/joint_commands` 接收 **Autolink 的 automsgs.sensor_msgs.JointState protobuf**，不是 ROS middleware。填写同等长度的 `name` 和 `position` 数组，可只更新部分关节。旋转关节单位为弧度，移动关节单位为米。只消费 position；velocity/effort 不是当前运动学位置控制的输入。未知名称、重复名称、非有限数值和超限位置会拒绝整条命令，原姿态保留。

状态通过 `/joint_states` 发布；关节位姿通过 `/tf` 动态发布，固定关节使用 `/tf_static`。命令在仿真主线程执行，网络线程和 Autolink 回调不会访问 Habitat 的 GL 上下文。Mimic 关节自动跟随源关节，用户应控制源关节。

直接调用后端也可控制：

```python
simulator.set_joint_positions({"joint_elbow": 0.6})
print(simulator.joint_states())
```

`habitat.robot.joint_positions` 可以设置初始姿态。`root_height` 设置 URDF 根节点离场景地面的高度；各 profile 提供展示默认值，可针对不同零姿态自行调整。默认固定底座，关节采用运动学位置控制。该功能支持模型展示与关节姿态操作；人形平衡、四足步态、接触动力学和行走策略需要另外接入控制器。五个模型均订阅 `/cmd_vel`（Autolink TwistStamped，读取 `twist.linear.x` 和 `twist.angular.z`）。0.5 秒无新指令后减速停止。支持前进、后退、原地转向和弧线运动；底座位姿、里程计和 TF 同步。TurtleBot3 与 Husky 配置了 `wheel_radius` 和 `wheel_separation`，轮子关节按差速关系随 `/cmd_vel` 滚动，`/joint_states` 中的轮速来自同一位置差分；底座仍由运动学积分推进，轮子不参与位姿解算。

速度上限按机型配置：Google 0.5 m/s、1.0 rad/s，G1 1.2 m/s、1.5 rad/s，Go2 2.0 m/s、2.0 rad/s，TurtleBot3 0.5 m/s、1.0 rad/s，Husky 1.0 m/s、1.5 rad/s。

`/joint_states` 的 `velocity` 由相邻两次位置写入的差值除以实际间隔得到。Habitat 是运动学后端，不会自己积分关节速度；仿真循环在发布后重新写入当前姿态，因此命令到达的那一帧速度非零，下一帧回到零。

## 支持模型

- Google：`google_robot`，15 个自由度，目录 `robot/google_robot/`。
- 宇树人形：`unitree_g1`，29 个自由度，目录 `robot/unitree_g1/`。
- 宇树四足：`unitree_go2`，12 个自由度，目录 `robot/unitree_go2/`。
- 两轮差速：`turtlebot3_burger`，2 个自由度（左右轮），目录 `robot/turtlebot3_burger/`。
- 四轮差速：`husky`，4 个自由度（前后左右轮），目录 `robot/husky/`。

TurtleBot3 与 Husky 的 `camera_link` 是普通 URDF 连杆系（x 前、y 左、z 上），`config/robots/*.yaml` 里用 `axes: body` 换算到 Habitat 相机；`base_footprint` 是 URDF 根节点，`root_height: 0` 表示它已经落在场景地面上。

## Google 外观与传感器

`robot/google_robot/meshes/habitat/` 是按参考照片重制的 Habitat 展示资源：偏白外壳 (236, 236, 236)、金色夹爪指 (219, 176, 61)、深色轮胎 (26, 26, 28)。原有通风口、字母和臂环细节保留，躯干上的金色服务面板改为与外壳一致的白色。所有材质都带 `KHR_materials_unlit`：Habitat-Sim 0.3.3 读取 glTF 底色纹理时不做 sRGB 解码，默认四灯组也会把受光外壳染成偏黄，unlit 材质才能让渲染颜色等于设定的调色板。

`head_sensor_panel.glb` 是叠加在头部外壳上的建模件：黑色面板、五个带银色镜圈的镜头（中间镜头为镀铬环）和两颗状态 LED。它作为 `link_head_tilt` 的第二个 `<visual>`，随头部旋转和俯仰一起运动。

重新生成外观需要 trimesh 和 Pillow，在 Habitat 之外运行：

```bash
python scripts/prepare_google_visuals.py --source /path/to/ManiSkill-GoogleRobot
```

`robot/google_robot/appearance.json` 记录调色板、unlit 决定和每个文件的面数。

URDF 保留真实机器人的传感器坐标系：`camera_center_optical_frame`、`camera_left_optical_frame`、`camera_right_optical_frame`、`imu.base`、`imu.neck`、八个 `time_of_flight.*` 和两个 `cliff.rear_*`。`config/robots/google_robot.yaml` 打开中心 RGB-D 和一组立体相机：

```yaml
    camera:
      enabled: true
      frame: camera_center_optical_frame
      follow_urdf: true
    stereo:
      enabled: true
      width: 640
      height: 480
      hfov_deg: 60.0
      rate_hz: 15.0
      baseline_m: 0.124
      left_frame: camera_left_optical_frame
      right_frame: camera_right_optical_frame
```

三个相机都跟随 `joint_head_pan` 和 `joint_head_tilt`。发布 `/camera/rgb/image_raw`、`/camera/depth/image_raw`、`/camera/camera_info` 以及 `/camera/left|right/image_raw`、`/camera/left|right/camera_info`。右目 `camera_info` 的 `P[3] = -fx * baseline`，符合校正立体惯例；左右图共用同一时间戳。`scripts/smoke_robot_sensors.py --scene <glb>` 校验话题、分辨率、frame_id、基线和视差。

IMU 和测距传感器按同一套 URDF 坐标系发布：

```yaml
    imu:
      enabled: true
      channel: /imu/data
      frame: imu.base
      rate_hz: 50.0
    range:
      enabled: true
      rate_hz: 10.0
      min_range: 0.02
      max_range: 4.0
      sensors:
        time_of_flight.right_1: /tof/right_1
        # ... 另外七个 time_of_flight.* 帧
        cliff.rear_right:
          channel: /cliff/rear_right
          max_range: 0.5
        cliff.rear_left:
          channel: /cliff/rear_left
          max_range: 0.5
```

`/imu/data` 的朝向是 IMU 安装坐标系在 `map` 下的姿态，线加速度含重力。姿态来自 URDF 链路旋转，因此 `imu.neck` 这类带俯仰的安装点不会退化成平面航向；角速度和比力也从 `base_link` 旋转到安装坐标系。`orientation_covariance[0] = 0` 表示姿态可用，`-1` 表示后端只提供平面航向。

Habitat-Sim 没有测距传感器，`range` 块让每个坐标系沿 URDF `+x` 轴投射射线，并跳过机器人自身的碰撞体。`cone_deg` 把单条射线扩成锥形光斑：Google 的八个 ToF 用 `cone_deg: 6.0`、`samples: 9`，在 6° 锥内按等面积螺旋取 9 条射线并取最近命中，细柱状障碍不会从两条射线之间漏过去；两个悬崖传感器保持单射线。`Range.field_of_view` 默认跟随 `cone_deg`，显式写 `fov` 时以 `fov` 为准。无回波按 ROS 惯例报 `inf`，所有光束共用同一时间戳，按最慢的 `rate_hz` 节流。

## 传感器时序

默认每个传感器都在采样当帧发布。需要模拟驱动缓冲时，给任意传感器块加 `latency_sec` 和 `jitter_sec`：

```yaml
    camera:
      enabled: true
      latency_sec: 0.035   # 固定延迟（秒）
      jitter_sec: 0.005    # 在 ±5 ms 内均匀抖动
```

延迟帧保留采样时刻的 `header.stamp`，只是晚到，因此下游看到的是真实的时间错位，而不是被改写的时间戳。抖动由 `habitat.sensors.timing_seed`（默认 0）播种，同一配置重放得到相同的到达顺序。同一帧的多条消息共用一次抽样：RGB、深度和 `CameraInfo` 一起延迟，立体左右目一起延迟，IMU 一次 burst 内的子样本保持先后顺序。`odom`、`/tf`、`/joint_states` 和 `/clock` 不进延迟队列，位姿与速度始终一致。上限是 `latency_sec ≤ 2.0`、`jitter_sec ≤ 1.0`，抖动不会让帧早于采样时刻到达。

## 宇树传感器

宇树模型使用 URDF 已有的安装坐标系，不额外假设不存在的 ToF 或悬崖传感器：

| 模型 | RGB-D | IMU | 2D 激光 |
|------|-------|-----|---------|
| `unitree_g1` | `d435_link`，640×480，HFOV 69.4° | `imu_in_torso`，50 Hz | `mid360_link`，360 束，30 m |
| `unitree_go2` | `front_camera`，640×480，HFOV 120° | `imu`，50 Hz | `laser_link`，360 束，30 m |

`d435_link` 和 `front_camera` 是普通 URDF 连杆坐标系（x 向前、y 向左、z 向上），不是 ROS 光学系。配置里用 `axes: body` 声明这一点，渲染时按 `Rz(-π/2) · Rx(π/2)` 转到 Habitat 相机系（-z 前、+y 上）；`axes` 缺省为 `optical`，表示连杆本身就是 x 右、y 下、z 前的光学系，Google 的 `camera_*_optical_frame` 走这条路径。两种写法都配合 `follow_urdf: true`，让相机随关节姿态和机身运动更新。Go2 的 `laser_link` 是配置层补的安装点，默认位于 URDF 根节点上方 0.1 m；`sensors.lidar_2d.height` 始终表示相对根节点的高度，`root_height` 只在射线世界坐标和 `odom→base` 变换中应用一次。G1 的 `mid360_link` 直接使用 URDF 安装位置。

激光射线使用 `sensors.lidar_2d.frame` 的实时位姿：`mid360_link` 挂在 `torso_link` 上，转动 `waist_yaw_joint` 后 `/scan` 的方向随之旋转，地图裁剪也使用同一朝向。URDF 里没有该坐标系时（Go2 的 `laser_link`）回退到根节点上方 `height` 的虚拟安装点。`/scan` 的 `scan_time` 等于 `1 / rate_hz`，`time_increment` 等于 `scan_time / (num_beams - 1)`，便于下游做运动补偿。

```yaml
    camera:
      enabled: true
      frame: d435_link
      follow_urdf: true
      axes: body          # URDF 连杆系；光学系连杆用 optical（默认）
    imu:
      enabled: true
      channel: /imu/data
      frame: imu_in_torso
    lidar_2d:
      enabled: true
      frame: mid360_link
```

## 预览场景与渲染

`habitat.path` 为空时，`habitat.scenario` 会生成程序化场景。五个机器人档案共用 `config/robots/_base.yaml` 的 `type: floor`：只生成 24 m × 24 m 的地面三角形，不放置障碍物，机器人站在实地上而不是悬空，相机也不会一眼看到地面边缘。查看器在未指定 `--scene` 时使用同样的地面，并读取 `config/robots/<profile>.yaml` 的 `habitat.robot`，预览的初始姿态和速度上限与正式配置一致。

地面 OBJ 旁边会写出同名 `.mtl`，包含中性灰的 `floor` 和浅灰的 `obstacle` 两种材质。没有材质文件时 Habitat 会用默认材质把地面渲染成棕色，看起来像泥土。

`habitat.render` 控制观感：

```yaml
  render:
    enable_hbao: true     # 接触阴影，底座和脚部不再像浮在地面上
    clear_color: [0.55, 0.57, 0.60, 1.0]  # 彩色传感器背景色
    lights:               # 可选，替换场景默认灯组
      - vector: [0.35, -0.9, -0.35, 0.0]   # w=0 方向光，w=1 点光
        color: [0.85, 0.86, 0.88]
```

`clear_color` 只作用于彩色传感器。空场景里 Habitat 默认把天空画成纯黑，地面边缘和背景糊在一起；设成中性灰后，RGB、立体和环视相机看到的是干净的摄影棚背景。真实 GLB 场景通常自带天空几何，可按需覆盖或去掉这一项。

`lights` 需要 `override_scene_light_defaults`，Habitat 才会接受 `set_light_setup`。未显式设置 `override_lights` 时，自定义灯组只作用于程序化场景；加载真实 GLB 时保留场景自带灯光。

Google 的 URDF 零位把手臂竖直举过头顶，不适合作为展示姿态。`config/robots/google_robot.yaml` 用 `joint_positions` 把肩部转到 3.0 rad、肘部 -0.9 rad，bicep 和 wrist 归零，手臂向前下方自然下垂且夹爪不穿地，夹爪指张开 0.6 rad。

G1 的零位把两条前臂水平前伸，看起来像端着东西。`config/robots/unitree_g1.yaml` 用 `joint_positions` 把肩部俯仰转到 0.10 rad、肩部横滚收到 ∓0.12 rad、肘部弯到 1.45 rad，手臂自然垂在身体两侧；`root_height` 仍按脚踝底面贴地，改动姿态不影响站立高度。查看器切换机型时会读取同一份 `habitat.robot`，“复位”按钮回到配置里的 home 姿态而不是 URDF 零位。

## 资源复现与验证

资源导入脚本固定两个上游 Git revision，逐文件核对 Git blob 哈希。它只下载所需 URDF、mesh 和伴随材质/纹理。G1 展示资源使用 GLB 转换和最多 5000 面的网格简化；Go2 保留全部原始三角面，不做简化，避免机身和腿部的薄壳表面出现缺失。纹理 UV 使用最近原始顶点采样，source.json 记录原始哈希、输出哈希和面数。关节定义、mesh 尺寸和坐标变换保留。Google 资源不简化。需要高精度外观或碰撞网格时，使用完整下载资源。URDF 中的 mesh 路径被改为本机器人目录内的相对路径，兼容 Habitat 和 Autoviz，不需要 ROS package URI 或 xacro。资源转换和路径变化记录在 source.json。

```bash
python scripts/import_robot_assets.py --destination /tmp/autosim-assets
python scripts/import_robot_assets.py --destination /tmp/google-assets --models google_robot
python scripts/prepare_habitat_meshes.py --source /tmp/autosim-assets --destination /tmp/autosim-display
python scripts/smoke_habitat_robots.py --output artifacts/robot-smoke
```

mesh 转换脚本需要 Python 3.10+ 和 trimesh、fast-simplification、pycollada、scipy、pillow；这些只是准备资源时的依赖。

导入目标目录必须不存在，以免覆盖已有模型；下载缓存可复用。`source.json` 记录每份复制资源的 SHA-256。Google 上游快照没有 LICENSE 文件，未给其资源重新指定许可证。宇树资源携带该仓库的 LICENSE。

`scripts/smoke_cmdvel_robots.py --profile unitree_go2` 验证真实 Autolink 指令传输、Habitat 底座移动、里程计/TF 和超时停止，使用独立验证频道。`scripts/smoke_robot_sensors.py --profile <name> --scene <glb>` 按模型实际启用的传感器校验 RGB-D、立体相机、IMU、ToF/悬崖和 2D 激光；Google 的立体基线为 0.124 m。

Smoke 检查每个模型是否真实加载、每个可独立控制的自由度能否移动，以及组合关节变化是否改变渲染图。输出控制前后的 PNG 和 `results.json`。多个模型分别在子进程运行，以便记录原生导入器崩溃。该检查不验证行走与碰撞动力学。

这五个模型只适配 Habitat，提供 URDF 与 mesh 资源；Isaac 后端不在适配范围内，对它们选择 `isaac` 会直接报错。
