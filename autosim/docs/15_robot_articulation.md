# Habitat 中的机器人模型与自由度控制

机器人资源位于 `robot/<profile>/`，每个目录包含 URDF、引用的展示 mesh、来源记录 `source.json` 和上游许可证（如有）。Google 使用保留轮子连续关节的 ManiSkill URDF，因此可以控制轮子、机械臂、夹爪和头部。宇树选用 G1 人形机器人（29 自由度）和 Go2 四足机器人（12 自由度），各自独立存放。

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
CONFIG=config/robots/unitree_go2.yaml scripts/run.sh both
```

控制频道 `/joint_commands` 接收 **Autolink 的 automsgs.sensor_msgs.JointState protobuf**，不是 ROS middleware。填写同等长度的 `name` 和 `position` 数组，可只更新部分关节。旋转关节单位为弧度，移动关节单位为米。只消费 position；velocity/effort 不是当前运动学位置控制的输入。未知名称、重复名称、非有限数值和超限位置会拒绝整条命令，原姿态保留。

状态通过 `/joint_states` 发布；关节位姿通过 `/tf` 动态发布，固定关节使用 `/tf_static`。命令在仿真主线程执行，网络线程和 Autolink 回调不会访问 Habitat 的 GL 上下文。Mimic 关节自动跟随源关节，用户应控制源关节。

直接调用后端也可控制：

```python
simulator.set_joint_positions({"joint_elbow": 0.6})
print(simulator.joint_states())
```

`habitat.robot.joint_positions` 可以设置初始姿态。`root_height` 设置 URDF 根节点离场景地面的高度；各 profile 提供展示默认值，可针对不同零姿态自行调整。默认固定底座，关节采用运动学位置控制。该功能支持模型展示与关节姿态操作；人形平衡、四足步态、接触动力学和行走策略需要另外接入控制器。三个模型均订阅 `/cmd_vel`（Autolink TwistStamped，读取 `twist.linear.x` 和 `twist.angular.z`），线速度上限 0.5 m/s，角速度上限 1.0 rad/s。0.5 秒无新指令后减速停止。支持前进、后退、原地转向和弧线运动；底座位姿、里程计和 TF 同步。

## 支持模型

- Google：`google_robot`，15 个自由度，目录 `robot/google_robot/`。
- 宇树人形：`unitree_g1`，29 个自由度，目录 `robot/unitree_g1/`。
- 宇树四足：`unitree_go2`，12 个自由度，目录 `robot/unitree_go2/`。

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

`/imu/data` 的朝向是平面航向四元数，线加速度含重力。Habitat-Sim 没有测距传感器，`range` 块让每个坐标系沿 URDF `+x` 轴投射一条射线，并跳过机器人自身的碰撞体：八个 ToF 用 4 m 量程，两个悬崖传感器用 0.5 m。无回波按 ROS 惯例报 `inf`，所有光束共用同一时间戳，按最慢的 `rate_hz` 节流。

## 宇树传感器

宇树模型使用 URDF 已有的安装坐标系，不额外假设不存在的 ToF 或悬崖传感器：

| 模型 | RGB-D | IMU | 2D 激光 |
|------|-------|-----|---------|
| `unitree_g1` | `d435_link`，640×480，HFOV 69.4° | `imu_in_torso`，50 Hz | `mid360_link`，360 束，30 m |
| `unitree_go2` | `front_camera`，640×480，HFOV 120° | `imu`，50 Hz | `laser_link`，360 束，30 m |

`d435_link` 和 `front_camera` 的 URDF 坐标轴已经符合 ROS 光学坐标系，`follow_urdf: true` 会让相机随关节姿态和机身运动更新。Go2 的 `laser_link` 是配置层补的安装点，默认位于 URDF 根节点上方 0.1 m；`sensors.lidar_2d.height` 始终表示相对根节点的高度，`root_height` 只在射线世界坐标和 `odom→base` 变换中应用一次。G1 的 `mid360_link` 直接使用 URDF 安装位置。

```yaml
    camera:
      enabled: true
      frame: d435_link
      follow_urdf: true
    imu:
      enabled: true
      channel: /imu/data
      frame: imu_in_torso
    lidar_2d:
      enabled: true
      frame: mid360_link
```

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

新 profile 提供 URDF 资源，Isaac 后端仍需要用户指定实际的 `robot.usd_robot`。
