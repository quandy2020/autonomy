# 8　由 URDF 建模

- 源码：`bolza/pinocchio/` · `bolza/perceptive/`
- 示例：`examples/mobile_manipulator` · `examples/legged_robot`
- 宏：`BOLZA_HAS_PINOCCHIO`；自碰撞另需 `BOLZA_HAS_COAL`

## 8.1　流程

1. 解析 URDF；可用 `model_information.removeJoints` 固定无关关节
2. 建动力学与末端运动学（解析导数或 CppAD）
3. 注册末端跟踪代价、关节限位、自碰撞
4. 装配 OCP / Rollout / MPC；频繁求值再加 `PreComputation`

## 8.2　质心模型（四足）

- 状态：归一化质心动量 · 基座 · 关节
- 输入：接触力旋量 · 关节速度
- 选项：FCD / SRBD

## 8.3　运动学

- 命名 frame：位置 · 姿态误差 · 速度
- 有缓存：解析 Pinocchio 接口
- 无缓存 / 快改：CppAD 变体

## 8.4　自碰撞

- URDF 碰撞体子集 + HPP-FCL
- 网格宜简化为圆柱 / 长方体
- 单独调裕度与权重
- 无 coal → 无 `mobile_manipulator_demo`
