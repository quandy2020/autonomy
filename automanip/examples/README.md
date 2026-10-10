# 示例

OCS2 模型各用一个进程跑 MPC 和 rollout，用 Autolink 把 URDF、关节、轨迹和目标发给 autoviz。小车倒立摆、双积分器、四旋翼、球上平衡不依赖 Pinocchio。四足和移动机械臂在找到 pinocchio 时编译，机械臂的自碰撞还需要 hpp-fcl。

默认求解器是 Gauss-Newton DDP。`ballbot_demo` 的第二个参数可以换成已移植的 SLP。

## 编译

求解器源码在 CppAD、Boost.Log 和 OpenMP 都找得到时才会编进 `libautomanip`，示例也只在这时生成。`legged_robot_demo` 还要有 pinocchio，`mobile_manipulator_demo` 还要有 hpp-fcl。CppAD 默认用同级仓库的头文件：

`ocs2/ocs2_thirdparty/include/cppad/cg.hpp`

路径不对时，配置加上 `-DAUTOMANIP_CPPAD_INCLUDE=<含 cppad/cg.hpp 的目录>`。

在仓库根目录：

```bash
cmake --build build/autonomy -j"$(nproc)" --target \
  cartpole_demo double_integrator_demo quadrotor_demo ballbot_demo \
  legged_robot_demo mobile_manipulator_demo

export PATH=$PWD/build/autonomy/bin:$PATH
export LD_LIBRARY_PATH=$PWD/build/autonomy/lib:$LD_LIBRARY_PATH
```

任务文件和 URDF 在编译时写进二进制，运行时不用再传配置路径。Cartpole、Quadrotor、Ballbot、四足和机械臂第一次求解会把动力学库生成到 `/tmp/automanip/<name>`。

## 运行

第一个参数是 MRT 步数。`0` 或省略表示一直跑，Ctrl+C 停止。任务里的 MRT 频率是 400 Hz，所以 `400` 大约是 1 秒仿真。

```bash
cartpole_demo                 # 一直跑
cartpole_demo 400             # 滚 400 步后退出
double_integrator_demo 400
quadrotor_demo 400
ballbot_demo 400              # DDP，默认
ballbot_demo 0 slp            # SLP，一直跑
ballbot_demo 400 ddp          # 显式指定 DDP
legged_robot_demo 400         # DDP，ANYmal C
legged_robot_demo 400 sqp
legged_robot_demo 400 ipm
mobile_manipulator_demo 400   # Franka，DDP
```

`ballbot_demo` 的第二个参数只接受 `ddp` 或 `slp`，`legged_robot_demo` 只接受 `ddp`、`sqp` 或 `ipm`，其它值退出码是 2。步态用 `examples/legged_robot/config/command/reference.info` 里的序列，不另开键盘节点。

也可以用 launch 拉起，launch 不带步数，进程会一直跑，求解器用 DDP：

```bash
autolink launch start src/autonomy/automanip/examples/launch/cartpole.launch
autolink launch start src/autonomy/automanip/examples/launch/double_integrator.launch
autolink launch start src/autonomy/automanip/examples/launch/quadrotor.launch
autolink launch start src/autonomy/automanip/examples/launch/ballbot.launch
autolink launch start src/autonomy/automanip/examples/launch/legged_robot.launch
autolink launch start src/autonomy/automanip/examples/launch/mobile_manipulator.launch
```

安装后的 launch 在 `share/automanip/launch/`。线程优先级设不成功时会打出 warning，求解仍继续。

## 在 autoviz 里看

和 RViz2 的 RobotModel 一样：描述来自话题，位姿来自 TF。直接启动 autoviz，固定坐标系用默认的 `map`，再加一个 RobotModel 显示。

| 属性 | 值 |
|---|---|
| Description Source | Topic |
| Description Topic | `/robot_description` |
| 固定坐标系 | `map` |

示例按 `robot_state_publisher` 的方式发布：`/robot_description` 是 URDF（`std_msgs/String`），`/joint_states` 是关节，`/tf` 是 `map` 到各个 link。先开 autoviz 或先开示例都可以，描述会周期性重发。MarkerArray 只画目标，Path 画预测轨迹。

| 进程 | 模型 | 改目标 |
|---|---|---|
| `cartpole_demo` | 小车倒立摆。导轨 `slideBar`，关节 `slider_to_cart`、`cart_to_pole`。固定坐标系用 `slideBar` 时和上游 RViz 一致 | `PoseStamped.x` 是小车位置 |
| `double_integrator_demo` | 导轨 `slideBar` 上的灰球 `cart` 和红球 `target`。关节 `slider_to_cart`、`slider_to_target`。固定坐标系用 `slideBar` 时和上游 RViz 一致 | `PoseStamped.x` 是位置 |
| `quadrotor_demo` | 四旋翼。关节 `x y z yaw pitch roll` | `PoseStamped` 的 xyz 是位置 |
| `ballbot_demo` | 球上平衡。蓝球 `ball`，灰色机体网格 `base`。关节 `jball_x`、`jball_y`、`jbase_z`、`jbase_y`、`jbase_x`。固定坐标系是 `map` | `PoseStamped` 的 xy 是地面位置 |
| `legged_robot_demo` | ANYmal C。根连杆 `base`，十二条腿关节。视觉网格和贴图与上游 RViz 相同。足端球、绿色接触力、黑色支撑多边形发在标记话题 | `PoseStamped` 的 xy 和偏航改躯干目标，高度保持不变 |
| `mobile_manipulator_demo` | Franka Panda，根连杆 `root`。视觉网格与上游 RViz 相同 | `PoseStamped` 是末端 `panda_hand_tcp` 的位姿 |

`<name>` 是 `cartpole`、`double_integrator`、`quadrotor`、`ballbot`、`legged_robot`、`mobile_manipulator`。

| 通道 | 消息 | 作用 |
|---|---|---|
| `/robot_description` | `std_msgs/String` | URDF，RobotModel 的 Description Topic |
| `/joint_states` | `sensor_msgs/JointState` | 与 URDF 关节同名 |
| `/tf` | `tf2_msgs/TFMessage` | `map` → link，RobotModel 用来摆模型 |
| `/<name>/path` | `nav_msgs/Path` | MPC 预测轨迹 |
| `/<name>/markers` | `visualization_msgs/MarkerArray` | 目标 |
| `/<name>/target_pose` | `geometry_msgs/PoseStamped` | 订阅。新消息会改目标，未写到的状态分量保持上次的值 |

URDF 在 `examples/<name>/urdf/<name>.urdf`。模型和关节只发全局话题 `/robot_description`、`/joint_states`，不再各发一份 `/<name>/` 副本。一次只跑一个示例，因为这两个话题和 `/tf` 是全局的。

## 没有放进来的模型

`perceptive_anymal` 还依赖 RaiSim 和地形，不在当前示例里。`legged_robot` 在找到 pinocchio 时编译，`mobile_manipulator` 还需要 hpp-fcl。距离场和网格插值在 `automanip/perceptive`。SQP 求解器在 `automanip/sqp`，二次子问题用离散 Riccati。
