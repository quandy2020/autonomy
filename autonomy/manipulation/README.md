# Manipulation

机械臂运动规划与执行栈。阶段：**固定工位 runtime 可用**（MoveIt 同构主路径 + Setup CLI lite）。

相对 [MoveIt2](https://github.com/ros-planning/moveit2)：算法与接口高度同构；**非**全量产品等价。

布局对齐 [autonomy/planning](../planning/)：根目录 Server/main；`common/` 接口；`planner/<algo>/` 分桶；`model/` / `motion/` / `dispatch/` 分层；文件尾 `AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN`。

## 类型约定（automsgs）

运行时状态 / 轨迹 / 位姿 / 场景物体 / 路径约束 **不以平行 POD 定义**，统一使用 automsgs 生成类型（见 `common/msg_types.hpp`）：

| 概念 | 类型 |
|------|------|
| 关节状态 | `sensor_msgs/JointState` |
| 关节轨迹 | `trajectory_msgs/JointTrajectory` |
| 位姿 | `geometry_msgs/Pose`（`PoseStamped`） |
| 伺服 | `Twist` / `TwistStamped` / `control_msgs/JointJog` |
| 场景物体 | `moveit_msgs/CollisionObject` / `AttachedCollisionObject` |
| 允许碰撞矩阵 | `moveit_msgs/AllowedCollisionMatrix`（场景内另有 O(1) lookup 缓存） |
| 路径约束 | `moveit_msgs/{Joint,Position,Orientation}Constraint` |

辅助：`common/joint_state_util.hpp`、`motion/scene/collision_object_util.hpp`（`MakeBoxObject` / `GetObjectPose`）、`motion/kinematics/pose_util.hpp`（`InterpolatePose`）。复用 `autonomy/common`（`LRUCache`、`Clamp`）与 `autolink/common/file.hpp`。

仍为 C++ 域对象（非 wire POD）：URDF/SRDF 模型、`PlanningScene` 运行时、`MotionPlanRequest` 外壳（内含 pb 字段 + `shared_ptr` 依赖）。

## 目录结构

```
manipulation/
├── manipulation_main.cpp / manipulation_server.* / manipulation_options.*
├── common/          # 类型别名、接口、plugin_ids、util
├── model/           # 原 core：机器人模型域
├── planner/         # <algo> + pipeline / constraint_samplers / optimize
├── motion/          # 原 runtime：场景·碰撞·运动学·动力学·执行·伺服
├── dispatch/        # capability + move_group_interface + action_server
├── setup/
└── conf/ launch/ dag/ proto/ benchmarks/
```

## 目标边界

| 目标 | 状态 |
|------|------|
| 产线固定工位 runtime | **完成（本模块）** |
| Setup GUI / Bullet / warehouse / RViz | **明确不做** |
| 驱动固件力矩施加 | **模块外**（订阅 `effort_command`） |
| VHACD / TRAC-IK / Octomap | FEATURE 可选 |

## 对齐进度（相对 MoveIt2）

| 能力 | 对齐度 | 现状 |
|------|--------|------|
| Pilz / TOTG / FCL / TEM | ~94% | TEM 场景校验 + EffortTracking |
| ompl_interface | ~92% | CSS + ParallelPlan + BiTRRT/EST |
| constraint_samplers | ~93% | Clamp Project + near-seed + 单测 |
| CHOMP / STOMP / Hybrid | ~90% | 分文件 + trajectory_optimize |
| kinematics | ~90% | KDL/IKFast/TRAC_IK + common::LRUCache |
| dynamics | ~90% | Pinocchio 重力/NLE → effort_command |
| Servo | ~93% | Twist/JointJog/PoseStamped + 接近减速 |
| Setup / Pipeline | ~90% | path_constraints 起点投影 |

**整体（固定工位）**：~93–95%。

## FEATURES

`fcl` / `ompl` / `ruckig` / `kdl` / `urdfdom` / `pinocchio` / `vhacd` / `trac_ik` / `octomap`

## 进程

- `autonomy.manipulation` · `/autonomy/manipulation/move`
- `autonomy.manipulation.setup`

## 非本模块职责

| 项 | 说明 |
|----|------|
| 驱动固件 | 订阅 `/arm_controller/effort_command`，施加力矩 |
| Setup GUI / Bullet / warehouse / RViz | **不做** |
