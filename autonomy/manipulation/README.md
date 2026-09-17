# Manipulation

机械臂运动规划与执行栈。阶段：**固定工位 runtime 可用**（MoveIt 同构主路径 + Setup CLI lite）。

相对 [MoveIt2](https://github.com/ros-planning/moveit2)：算法与接口高度同构；**非**全量产品等价。

布局对齐 [autonomy/planning](../planning/)：根目录 Server/main；`common/` 接口；`planner/<algo>/` 仅算法分桶；`pipeline/` / `constraints/` 为非算法规划基础设施；`model/` / `motion/` / `dispatch/` 分层；文件尾 `AUTOLINK_PLUGIN_MANAGER_REGISTER_PLUGIN`。

## 类型约定（automsgs）

运行时状态 / 轨迹 / 位姿 / 场景物体 / 路径约束 **不以平行 POD 定义**，直接使用 `automsgs::msgs::*` 与 `autonomy/manipulation/proto`：

| 概念 | 类型 |
|------|------|
| 关节状态 | `automsgs::msgs::sensor_msgs::JointState` |
| 关节轨迹 | `automsgs::msgs::trajectory_msgs::JointTrajectory` |
| 位姿 | `automsgs::msgs::geometry_msgs::Pose`（`PoseStamped`） |
| 伺服 | `Twist` / `TwistStamped` / `control_msgs::JointJog` |
| 场景物体 | `moveit_msgs::CollisionObject` / `AttachedCollisionObject` |
| 允许碰撞矩阵 | `moveit_msgs::AllowedCollisionMatrix`（场景内另有 O(1) lookup 缓存） |
| 路径约束 | `moveit_msgs::{Joint,Position,Orientation}Constraint` |
| 规划请求线体 | `proto::MotionPlanRequest`（经 `planner::MotionPlanRequest::pb`） |
| 规划响应 / IK / 碰撞结果 | `proto::{MotionPlanResponse,InverseKinematicsOptions,CollisionResult,DistanceResult}` |

`common/`：仅 `*_interface.hpp`（`manipulation::common`，对齐 `control/common`；域命名空间保留 using 别名）。

`MotionPlanRequest` 在 `pipeline/motion_plan_request.*`（`pb` + 运行时 `shared_ptr`；命名空间仍为 `planner::`）。插件别名在根目录 `plugin_ids.hpp` / `plugin_registry.cpp`。

辅助：`model/joint_state_utilities.hpp`、`motion/scene/collision_object_helpers.hpp`、`motion/kinematics/pose_interpolation.hpp`。复用 `autonomy/common` 与 `autolink/common/file.hpp`。

## 目录结构

```
manipulation/
├── manipulation_main.cpp / manipulation_server.* / manipulation_options.*
├── plugin_ids.hpp / plugin_registry.cpp
├── common/          # 仅 *_interface.hpp
├── proto/           # manipulation 域 protobuf（非 automsgs wire）
├── model/           # 机器人模型域 + joint_state_utilities / link_forward_kinematics
├── planner/         # 仅算法：pilz / ompl / chomp / stomp
├── utils/           # trajopt 轨迹优化核（chomp/stomp 共用）
├── pipeline/        # PlanningPipeline / adapters / time parameterization / motion_plan_message_conversion / MotionPlanRequest
├── constraints/     # 路径约束检查与采样
├── motion/          # 场景·碰撞·运动学·动力学·执行·伺服
├── dispatch/        # capability + move_group_interface + action_server
├── setup/
└── conf/ launch/ dag/
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
| Pilz / time parameterization / FCL / TEM | ~94% | TEM 场景校验 + EffortTracking |
| ompl_interface | ~92% | CSS + ParallelPlan + BiTRRT/EST |
| constraints | ~93% | Clamp Project + near-seed + 单测 |
| CHOMP / STOMP | ~90% | 分文件 + trajopt 优化核 |
| kinematics | ~90% | KDL/IKFast/TRAC_IK + common::LRUCache |
| dynamics | ~90% | Pinocchio 重力/NLE → effort_command |
| CartesianServo | ~93% | Twist/JointJog/PoseStamped + 接近减速 |
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
