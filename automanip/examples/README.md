# 示例

OCS2 模型各用一个进程跑 MPC 和 rollout，用 Autolink 把 URDF、关节、轨迹和目标发给 autoviz。小车倒立摆、双积分器、四旋翼、球上平衡不依赖 Pinocchio。四足和移动机械臂在找到 pinocchio 时编译，机械臂的自碰撞还需要 hpp-fcl。

## OCP 用法（`bolza::Problem`）

`double_integrator` / `cartpole` / `quadrotor` / `ballbot` 各用 `{name}.hpp` / `{name}.cpp`，经 [`bolza::Problem`](../bolza/bolza/problem.hpp) 装动力学与二次代价：

```cpp
problem.SetLinearDynamics(A, B);          // 或 SetDynamics(...)
problem.AddQuadraticCost(Q, R);
problem.AddQuadraticFinalCost(Qf);
problem.SetInitialState(x0);
problem.SetTarget(x_star);
// Autolink demo：GaussNewtonDDPMPC(..., problem.GetOptimalControlProblem(), ...)
// 无可视化：bolza::Solve(options, &problem, &summary)  →  tutorial/solve_double_integrator.cpp
```

纯库教程目标：`bolza_tutorial_di`。`legged_robot` / `mobile_manipulator` 仍用底层 `OptimalControlProblem`（切换参考 / 软约束较多）。

默认求解器是 Gauss-Newton DDP。`ballbot_demo` 的第二个参数可以换成已移植的 SLP。

## 编译

`libbolza` 在 CppAD、OpenMP、Pinocchio、GLPK 等依赖齐备时编入；示例依赖 `bolza` 目标。`legged_robot_demo` 还要有 pinocchio，`mobile_manipulator_demo` 还要有 hpp-fcl / coal。库本身**不依赖 Boost**（INFO / 日志 / 积分均为自研或 C++17）。

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

除移动机械臂外，任务文件和 URDF 在编译时写进二进制，运行时不用再传配置路径。`mobile_manipulator_demo` 按机型名在运行时选择 `examples/mobile_manipulator/config/<robot>/task.info` 和 `bolza/examples/assets/mobile_manipulator` 下的 URDF。Cartpole、Quadrotor、Ballbot、四足和机械臂第一次求解会把动力学库生成到 `/tmp/automanip/<name>`；机械臂再按机型分到 `/tmp/automanip/mobile_manipulator/<robot>`。

## 运行

第一个参数是 MRT 步数。`0` 或省略表示一直跑，Ctrl+C 停止。任务里的 MRT 频率是 400 Hz，所以 `400` 大约是 1 秒仿真。`mobile_manipulator_demo` 还可以带机型名，和步数的先后不限；省略机型时是 Franka。

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
mobile_manipulator_demo              # Franka Panda，一直跑
mobile_manipulator_demo 400          # Franka，400 步
mobile_manipulator_demo pr2          # Willow Garage PR2
mobile_manipulator_demo ridgeback    # Clearpath Ridgeback + UR5
mobile_manipulator_demo kinova       # Kinova Jaco2，j2n6s300
mobile_manipulator_demo j2n7         # Kinova Jaco2，j2n7s300
mobile_manipulator_demo mabi         # Mabi-Mobile
```

`ballbot_demo` 的第二个参数只接受 `ddp` 或 `slp`，`legged_robot_demo` 只接受 `ddp`、`sqp` 或 `ipm`，其它值退出码是 2。步态用 `examples/legged_robot/config/command/reference.info` 里的序列，不另开键盘节点。机械臂机型名还可以写成 `panda`、`ridgeback_ur5`、`j2n6`、`j2n6s300`、`j2n7s300`、`mabi_mobile`。不认识的名字直接退出。

也可以用 launch 拉起，launch 不带步数，进程会一直跑，求解器用 DDP：

```bash
autolink launch start src/autonomy/automanip/examples/launch/cartpole.launch
autolink launch start src/autonomy/automanip/examples/launch/double_integrator.launch
autolink launch start src/autonomy/automanip/examples/launch/quadrotor.launch
autolink launch start src/autonomy/automanip/examples/launch/ballbot.launch
autolink launch start src/autonomy/automanip/examples/launch/legged_robot.launch
autolink launch start src/autonomy/automanip/examples/launch/mobile_manipulator.launch
autolink launch start src/autonomy/automanip/examples/launch/mobile_manipulator_pr2.launch
autolink launch start src/autonomy/automanip/examples/launch/mobile_manipulator_ridgeback.launch
autolink launch start src/autonomy/automanip/examples/launch/mobile_manipulator_kinova.launch
autolink launch start src/autonomy/automanip/examples/launch/mobile_manipulator_kinova_j2n7.launch
autolink launch start src/autonomy/automanip/examples/launch/mobile_manipulator_mabi.launch
```

安装后的 launch 在 `share/automanip/launch/`。线程优先级设不成功时会打出 warning，求解仍继续。

## 在 autoviz 里看

和 RViz2 的 RobotModel 一样：描述来自话题，位姿来自 TF。直接启动 autoviz，固定坐标系用默认的 `map`，再加一个 RobotModel 显示。

| 属性 | 值 |
|---|---|
| Description Source | Topic |
| Description Topic | `/robot_description` |
| 固定坐标系 | `map` |

示例按 `robot_state_publisher` 的方式发布：`/robot_description` 是 URDF（`std_msgs/String`，网格 URI 尽量改写为绝对路径以便 Autoviz 加载），`/joint_states` 是关节，`/tf` 是 `map` 到 URDF 根连杆及活动 link。先开 autoviz 或先开示例都可以，描述会周期性重发。机体几何只走 RobotModel；MarkerArray 画目标/足端等叠加物（并对曾用 Marker 画过的机体发 `DELETE`），Path 画预测轨迹。

| 进程 | 模型 | 改目标 |
|---|---|---|
| `cartpole_demo` | 小车倒立摆。导轨 `slideBar`，关节 `slider_to_cart`、`cart_to_pole`。固定坐标系用 `slideBar` 时和上游 RViz 一致 | `PoseStamped.x` 是小车位置 |
| `double_integrator_demo` | 导轨 `slideBar` 上的灰球 `cart` 和红球 `target`。关节 `slider_to_cart`、`slider_to_target`。固定坐标系用 `slideBar` 时和上游 RViz 一致 | `PoseStamped.x` 是位置 |
| `quadrotor_demo` | 四旋翼。单个连杆 `base`，黑色网格 `quadrotor.obj`。固定坐标系是 `map`。悬停高度 1 m，跟踪三维位置和偏航 | autoviz 的 Nav Goal。xy 和偏航跟点击，高度在 0.5 m 到 3.0 m 之间随机 |
| `ballbot_demo` | 球上平衡。蓝球 `ball`，灰色机体网格 `base`。关节 `jball_x`、`jball_y`、`jbase_z`、`jbase_y`、`jbase_x`。固定坐标系是 `map`。机体上方有速度文字 | autoviz 的 Nav Goal，或 `PoseStamped` 的 xy 和偏航 |
| `legged_robot_demo` | ANYmal C。根连杆 `base`，十二条腿关节。视觉网格和贴图与上游 RViz 相同（DAE 内嵌贴图）。足端球、绿色接触力、黄色压力中心、黑色支撑多边形发在标记话题 | `PoseStamped` 的 xy 和偏航改躯干目标，高度保持不变 |
| `mobile_manipulator_demo` | 移动机械臂，固定坐标系 `map`。`franka`：Panda，基座 `root`，末端 `panda_hand_tcp`。`pr2`：轮式，基座 `base_footprint`，末端 `r_gripper_tool_frame`。`ridgeback`：Ridgeback+UR5，基座 `base_link`，末端 `ur_arm_tool0`。`kinova` / `j2n7`：Jaco2，基座 `root`。`mabi`：轮式，基座 `base`，末端 `WRIST_2`，几何是长方体。绿色球是当前末端目标，红色折线是基座轨迹，蓝色折线是末端轨迹。灰色方块是 6 自由度目标标记，初始在 `(0, 0, 1)` | 和 [OCS2 示例](https://leggedrobotics.github.io/ocs2/robotic_examples.html) 一样：Interact 拖动只摆标记，右键 **Send target pose** 后才把该位姿发给 MPC，机械臂从这时开始动。末端参考从当前位姿按约 0.8 m/s、2.0 rad/s 走到目标，不会在一个规划窗口里甩到位。`PoseStamped` 仍是位置加四元数。开环初始目标是 `(1, 0, 1)`，在发出标记前不会执行 |

`<name>` 是 `cartpole`、`double_integrator`、`quadrotor`、`ballbot`、`legged_robot`、`mobile_manipulator`。

| 通道 | 消息 | 作用 |
|---|---|---|
| `/robot_description` | `std_msgs/String` | URDF，RobotModel 的 Description Topic |
| `/joint_states` | `sensor_msgs/JointState` | 与 URDF 关节同名 |
| `/tf` | `tf2_msgs/TFMessage` | Ballbot：`map`→`world` / `base` / `command`。Cartpole：`map`→`world`→导轨→小车→摆杆。双积分器：导轨、小车、目标。四旋翼：`map`→`base` / `command`。移动机械臂只发 `map`→基座、仍可动的连杆和末端，另有 `map`→`command`；相机、外壳和 `removeJoints` 锁住的关节不发坐标轴。四足只发 `map`→`base` 和每条腿的髋、大腿、小腿、足端 |
| `/<name>/path` | `nav_msgs/Path` | MPC 预测轨迹。移动机械臂这条是基座轨迹 |
| `/<name>/markers` | `visualization_msgs/MarkerArray` | 目标。移动机械臂还有基座和末端折线 |
| `/simple_marker/update` | `visualization_msgs/InteractiveMarkerUpdate` | 移动机械臂发布。标记名 `Goal`。拖动后的位姿会写回这条通道，避免标记弹回初始点 |
| `/simple_marker/feedback` | `visualization_msgs/InteractiveMarkerFeedback` | 移动机械臂订阅。拖动只更新标记。菜单项 id 1 “Send target pose” 才把位姿当成末端目标并开始执行 |
| `/<name>/target_pose` | `geometry_msgs/PoseStamped` | 订阅。新消息会改目标，未写到的状态分量保持上次的值 |
| `/goal_pose` | `geometry_msgs/PoseStamped` | 订阅。autoviz 工具栏 Nav Goal 的默认话题，和上面同一套目标 |
| `/cmd_vel` | `geometry_msgs/TwistStamped` | Ballbot 发布。航向坐标系 `dummy_base1` 下的平面速度和偏航角速度，autoviz 用 TwistStamped 画箭头 |

URDF 和网格在 `bolza/examples/assets`（cartpole、double_integrator、quadrotor、ballbot、`anymal_c`、`mobile_manipulator` 下的 franka、pr2、ridgeback_ur5、kinova、mabi_mobile）。示例通过 `LoadExampleRobotDescription` 解析 URDF：优先编译期路径，失败则回退 `AUTOMANIP_ROBOTIC_ASSETS` / `BOLZA_ROBOTIC_ASSETS`；把 `file://$(find …)` 规范成 `package://`，并在 sibling `meshes/` 存在时把 `package://ocs2_robotic_assets/resources/<robot>/meshes/…`（含 Franka 嵌套路径）改写为绝对路径。移动机械臂按机型直接取 assets 里的那份 URDF，再做同样的网格改写。Mabi-Mobile 的 URDF 只有长方体，没有网格文件。未改写的 package URI 仍可由 Autoviz `MeshResourceResolver` 解析。模型和关节只发全局话题 `/robot_description`、`/joint_states`。一次只跑一个示例，因为这两个话题和 `/tf` 是全局的。

## 没有放进来的模型

`perceptive_anymal` 还依赖 RaiSim 和地形，不在当前示例里。`legged_robot` 在找到 pinocchio 时编译，`mobile_manipulator` 还需要 hpp-fcl。距离场和网格插值在 `automanip/perceptive`。SQP 求解器在 `automanip/sqp`，二次子问题用离散 Riccati。
