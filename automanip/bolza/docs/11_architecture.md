# 11　软件结构

## 11.1　包布局

- `automanip/` — 规划 / 执行主栈
- `bolza/` — `libbolza`
- `examples/` — MPC 示例进程

## 11.2　库分层

```text
mpc/ → ddp | sqp | ipm | slp | qp/ → oc/ → core/
       frank_wolfe · bilevel · switched_model
       tools · pinocchio · perceptive · mpcnet · python
```

- 装配入口：`tools/`（`RobotInterface`）

## 11.3　常用类型

- `OptimalControlProblem` · `RolloutBase` · `ReferenceManager`
- 各 `*_MPC` · `SystemDynamicsBase`

## 11.4　运行路径

1. 更新参考（`TargetTrajectories` / mode schedule）
2. `setCurrentObservation` / `advanceMpc`
3. `PrimalSolution` → rollout 或下发

- 频率：`mpcDesiredFrequency` · `mrtDesiredFrequency`

## 11.5　特性宏

- `BOLZA_HAS_HPIPM`
- `BOLZA_HAS_FRANK_WOLFE`
- `BOLZA_HAS_SWITCHED_MODEL`
- `BOLZA_HAS_PINOCCHIO`
- `BOLZA_HAS_COAL`
- `BOLZA_HAS_ONNX`
