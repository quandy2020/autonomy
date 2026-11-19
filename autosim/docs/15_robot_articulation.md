# Habitat 中的机器人模型与自由度控制

机器人资源位于 `robot/<profile>/`，每个目录包含 URDF、引用的展示 mesh、来源记录 `source.json` 和上游许可证（如有）。Google 使用保留轮子连续关节的 ManiSkill URDF，因此可以控制轮子、机械臂、夹爪和头部。宇树选用 G1 人形机器人（29 自由度）和 Go2 四足机器人（12 自由度），各自独立存放。

## 查看与操作

在已有 SpaceHero 容器中运行：

```bash
docker exec -it \
  -w /workspace/autonomy/src/autonomy/autosim \
  -e PYTHONPATH=/workspace/autonomy/install/autonomy/lib/python \
  SpaceHero /opt/venv/bin/python3 -m autosim.robot.viewer \
  --profile google_robot --host 0.0.0.0 --port 8765
```

浏览器访问 `http://192.168.66.124:8765`。顶部型号选择可在同一 Habitat 场景中切换机器人。右侧为所有关节的滑块和数值输入，支持重置姿态、调整观察方位、仰角和距离。图像由 Habitat-Sim 渲染，浏览器只是显示画面和传送控制命令。默认监听 `127.0.0.1`；上述 LAN 示例依赖容器使用 host 网络。

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

为每个模型提供 `config/robots/<profile>.yaml`。例如：

```bash
CONFIG=config/robots/unitree_go2.yaml scripts/run.sh sim
```

控制频道 `/joint_commands` 接收 **Autolink 的 automsgs.sensor_msgs.JointState protobuf**，不是 ROS middleware。填写同等长度的 `name` 和 `position` 数组，可只更新部分关节。旋转关节单位为弧度，移动关节单位为米。只消费 position；velocity/effort 不是当前运动学位置控制的输入。未知名称、重复名称、非有限数值和超限位置会拒绝整条命令，原姿态保留。

状态通过 `/joint_states` 发布；关节位姿通过 `/tf` 动态发布，固定关节使用 `/tf_static`。命令在仿真主线程执行，网络线程和 Autolink 回调不会访问 Habitat 的 GL 上下文。Mimic 关节自动跟随源关节，用户应控制源关节。

直接调用后端也可控制：

```python
simulator.set_joint_positions({"joint_elbow": 0.6})
print(simulator.joint_states())
```

`habitat.robot.joint_positions` 可以设置初始姿态。`root_height` 设置 URDF 根节点离场景地面的高度；各 profile 提供展示默认值，可针对不同零姿态自行调整。默认固定底座，关节采用运动学位置控制。该功能支持模型展示与关节姿态操作；人形平衡、四足步态、接触动力学和行走策略需要另外接入控制器。既有 `/cmd_vel` 仍为平面运动接口。

## 支持模型

- Google：`google_robot`，15 个自由度，目录 `robot/google_robot/`。
- 宇树人形：`unitree_g1`，29 个自由度，目录 `robot/unitree_g1/`。
- 宇树四足：`unitree_go2`，12 个自由度，目录 `robot/unitree_go2/`。

## 资源复现与验证

资源导入脚本固定两个上游 Git revision，逐文件核对 Git blob 哈希。它只下载所需 URDF、mesh 和伴随材质/纹理。当前宇树展示资源使用 GLB 转换和最多 5000 面的网格简化；纹理 UV 使用最近原始顶点采样，source.json 记录原始哈希、输出哈希和面数。关节定义、mesh 尺寸和坐标变换保留。Google 资源不简化。需要高精度外观或碰撞网格时，使用完整下载资源。URDF 中的 mesh 路径被改为本机器人目录内的相对路径，兼容 Habitat 和 Autoviz，不需要 ROS package URI 或 xacro。资源转换和路径变化记录在 source.json。

```bash
python scripts/import_robot_assets.py --destination /tmp/autosim-assets
python scripts/import_robot_assets.py --destination /tmp/google-assets --models google_robot
python scripts/prepare_habitat_meshes.py --source /tmp/autosim-assets --destination /tmp/autosim-display
python scripts/smoke_habitat_robots.py --output artifacts/robot-smoke
```

mesh 转换脚本需要 Python 3.10+ 和 trimesh、fast-simplification、pycollada、scipy、pillow；这些只是准备资源时的依赖。

导入目标目录必须不存在，以免覆盖已有模型；下载缓存可复用。`source.json` 记录每份复制资源的 SHA-256。Google 上游快照没有 LICENSE 文件，未给其资源重新指定许可证。宇树资源携带该仓库的 LICENSE。

Smoke 检查每个模型是否真实加载、每个可独立控制的自由度能否移动，以及组合关节变化是否改变渲染图。输出控制前后的 PNG 和 `results.json`。多个模型分别在子进程运行，以便记录原生导入器崩溃。该检查不验证行走与碰撞动力学。

新 profile 提供 URDF 资源，Isaac 后端仍需要用户指定实际的 `robot.usd_robot`。
