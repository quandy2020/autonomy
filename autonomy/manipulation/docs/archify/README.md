# Manipulation Archify 图集

机械臂（Autonomy Manipulation）架构与流程的 Archify 交付物。每张图含：

- `*.json`：IR 源
- `*.html`：可交互独立页面（主题切换 / 聚焦 / 导出 PNG·SVG 等）
- `*.visual-check.*.png`：light/dark × 1440×900 / 2048×1320 截图

## 图索引

| 类型 | 文件 | 说明 |
|------|------|------|
| Architecture | [manipulation-runtime.architecture.html](./manipulation-runtime.architecture.html) | 运行时组件：Client → Action → ManipServer → Pipeline/Planner → TEM → JT Controller → Arm；旁路 Cartesian Servo |
| Workflow | [manipulation-plan-execute.workflow.html](./manipulation-plan-execute.workflow.html) | 规划—执行工作流主路径与失败/抢占分支 |
| Sequence | [manipulation-plan-execute.sequence.html](./manipulation-plan-execute.sequence.html) | PlanAndExecute 调用时序（含 IK 预处理） |
| Dataflow | [manipulation-planning.dataflow.html](./manipulation-planning.dataflow.html) | JointState/URDF → Scene/Model/Request → Planner → Trajectory → TEM → Driver |
| Lifecycle | [manipulation-action.lifecycle.html](./manipulation-action.lifecycle.html) | Action 状态：Idle → Accepted → Planning → Executing → Succeeded；Aborted / Cancelled |

## 主流程（摘要）

```
Client / MoveGroup
  → Action Server (/manipulation/move)
  → ManipulationServer (Capability: plan_and_execute)
  → PlanningPipeline (pre adapters → Planner → post adapters)
       ↳ Planner Plugins: Pilz / OMPL / CHOMP / STOMP
       ↳ 支撑：RobotModel · PlanningScene · Kinematics
  → Trajectory Execution Manager (TEM)
  → JointTrajectoryController (topic)
  → Arm Hardware

旁路：ManipulationServer → CartesianServo → JT Controller（实时闭环）
```

## 重新生成

```bash
ARCHIFY=.tools/archify/archify
OUT=autonomy/manipulation/docs/archify
node $ARCHIFY/bin/archify.mjs deliver architecture $OUT/manipulation-runtime.architecture.json $OUT/manipulation-runtime.architecture.html --quality showcase
# 同理 workflow / sequence / dataflow / lifecycle
ARCHIFY_CHROME_NO_SANDBOX=1 node $ARCHIFY/bin/archify.mjs visual-check $OUT/<file>.html --json
```
