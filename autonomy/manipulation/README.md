# Manipulation

机械臂运动规划与执行栈。目录对齐 [MoveIt 2](https://github.com/moveit/moveit2) 能力切分，交付形态为 **A+②**：全模块接口 + 可运行自研后端；OMPL/FCL/Ruckig/urdfdom 可选 FEATURES。

## API 文档

公开头文件使用 Doxygen 注释（`@brief` / `@param` / `@return`）。

## 布局

| 目录 | 职责 |
|------|------|
| `core/` | RobotModel / ErrorCode / metrics |
| `scene/` | PlanningScene / Monitor / msg_convert |
| `kinematics/` | FK/IK 插件 |
| `collision/` | 碰撞检测插件 |
| `planning/` | Planners / Pipeline / Trajectory / constraint_samplers |
| `execution/` | Controllers / TrajectoryExecutionManager |
| `server/` | ManipulationServer / MoveGroupInterface / Capabilities |
| `servo/` | DLS 伺服 |
| `plugins.hpp` | Autolink PluginManager 注册与 `CreatePlugin` |

## 成熟度（A+②）

| 能力 | 状态 |
|------|------|
| Autolink plugins / ErrorCode | 有（planner/kin/collision/adapter/capability） |
| RobotModel / Group / SRDF | Simple* + URDF limits + LinkFk 树 |
| Kinematics | stub / KDL / ikfast壳 / cached |
| Planners | joint_interpolation / cartesian / rrt_connect / pilz_{ptp,lin,circ} / CHOMP·STOMP（工业 lite） / hybrid；ompl 可选 |
| Collision | AABB（box/sphere/cylinder/meshAABB + self）；FCL 可选；octomap 占用点 |
| PlanningScene / Monitor | SceneDiff + ACM + Attach + ClearWorld/Octomap + PointCloud 占用 |
| Proto | moveit_msgs MotionPlan（含 position/orientation constraints） |
| Mimic | URDF `<mimic>` → LinkFk / RobotState |
| Adapters | Autolink 插件链：fix_start / TOTG / validate / check_constraints |
| Constraints | joint + position/orientation（proto 贯通；路径 FK 检查） |
| ACM | SRDF `disable_collisions` → PlanningScene |
| Execution | TEM Execute(replace) + DeviationHook（joint_states 偏差门控） |
| Capabilities | plan/execute/cartesian/fk_ik/scene + state_validation / query_planners / clear_* / validate_trajectory / get_urdf / save_load_geometry |
| Servo | DLS + 奇异/限速/关节限/碰撞安全门 |
| Task | Action Goal：`motion_plan` + `replace_execution`；Result 含 trajectory |

## FEATURES

| FEATURE | 宏 | 缺省行为 |
|---------|-----|----------|
| `kdl` | `AUTONOMY_HAS_KDL` | StubKinematics |
| `fcl` | `AUTONOMY_HAS_FCL` | AABB detector |
| `ompl` | `AUTONOMY_HAS_OMPL` | stub planner（头文件探测，避免 omplConfig→Boost；需可用 `libboost_system`） |
| `ruckig` | `AUTONOMY_HAS_RUCKIG` | TOTG（adapter 优先 Ruckig；Homebrew 无 formula 时保持 no-op） |
| `urdfdom` | `AUTONOMY_HAS_URDFDOM` | 正则 URDF |

真后端：`OmplPlanner`（RRTConnect）、`FclCollisionDetector`、`ApplyRuckig`、`ChompPlanner`（稠密 \(A^\top A\) 度量逆 + 软障碍势）/ `StompPlanner`、`PilzLin/Circ`、`HybridPlanner`。  
占位：`ikfast`；octomap binary 需 liboctomap（当前非 binary 索引三元组）。

## 进程 / 通道

- 二进制：`autonomy.manipulation`
- Action：`/autonomy/manipulation/move`
- 轨迹：`/arm_controller/joint_trajectory`

## 不做

Setup Assistant / warehouse / RViz / 完整 liboctomap binary 解码。
