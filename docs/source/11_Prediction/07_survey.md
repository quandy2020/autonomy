# 7. 预测综述

确认「车上有没有预测模块、该用什么替代」。

边界说明 + 包/代码选型；公式见 [§3](03_math.md)，可调用实现见 [§6](06_motion_models.md)。

| 你的目标 | 建议 |
|----------|------|
| 短期 demo 导航 | costmap，不等预测 |
| 离线滚轨迹 | motion_model + 自写 KF |
| 产品级多障碍预测 | 自研模块或等 Server 接线 |

### 仓库里实际有的

| 内容 | 路径 |
|------|------|
| 运动模型 | `autonomy/common/motion_model/`（Linear、DiffDrive、Stationary） |
| 卡尔曼滤波 | `autonomy/common/state_estimation/kalman_filter/` |
| PredictionServer | 头文件 + 空 `.cpp`，**无** Start/订阅/Writer |
| prediction 配置 proto | 预留字段，主进程未加载驱动 |

### 没有的（勿按文档旧图实现业务）

- TrackManager、匈牙利关联、CTRV 类、学习型/博弈预测  
- Perception detections → Prediction 自动管线  
- Planning 侧 `SetDynamicObstacles(trajectories)` 一类接口

### 目标管线（未闭合）

```text
Perception → Prediction → Planning
```

当前：**Perception（base/follow）与 Planning 之间无预测层**；动态性靠传感器刷新 costmap。

### 常见坑

| 误解 | 事实 |
|------|------|
| prediction 目录=完整模块 | 基本是壳 |
| Lua motion_model 字符串 | 无读取逻辑 |
| 必须 CTRV 才能预测 | CV Linear 即可做实验 |

### 延伸阅读

[§1 概览](01_overview.md) · [§5 架构](05_architecture.md)
