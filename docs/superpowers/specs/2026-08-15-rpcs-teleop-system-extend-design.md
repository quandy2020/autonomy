# RPCs 补齐：Teleop + System 管控

日期：2026-08-15  
状态：已落地  
范围：`teleop.proto`、`system.proto`、`common.proto`、README

## 新增

- `TeleopService.Drive` — bidi 速度遥控  
- `ClearEmergencyStop` / `CancelAllGoals` / `GetActiveGoal` / `GetCapabilities`  
- Code：`1003`、`1200–1203`
