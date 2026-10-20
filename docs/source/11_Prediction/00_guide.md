# 0. Prediction 指南

## 背景

导航要躲动态障碍，直觉是「先预测对方轨迹再规划」。仓库里也有 `prediction/` 与配置壳，但现状是：

- `PredictionServer` **仍是壳**，未接到 `Autonomy::Start()` 完整流水线。  
- 车上导航主路径仍是 **costmap → Planning → Control**，不依赖预测话题。  
- 若按文档里的 CTRV / 学习型方法去对接，会找不到实现。

## 思路

把本章当成 **可复用积木说明书**，不是「已上车预测产品」：

- 真能调的是 `autonomy/common/motion_model/`（Linear/CV、DiffDrive、Stationary）和卡尔曼相关代码。  
- 实验或工具里自行嵌模型即可；规划侧现在 **不必等** 预测上线。  
- 综述只写边界，避免把未实现算法写成已交付。

要调模型 → [06 运动模型](06_motion_models.md)。要看壳还缺什么 → [01](01_overview.md)、[05](05_architecture.md)。


## 步骤

### 1. 按问题选页

| 文档 | 解决什么 |
|------|----------|
| [01 概览](01_overview.md) | 目标形态 vs 现状 |
| [02 快速开始](02_quickstart.md) | 能跑的最小调用（若有） |
| [03 数学](03_math.md) | 模型公式 |
| [04 用法](04_usage.md) | Options / 接口边界 |
| [05 架构](05_architecture.md) | 目标分层 vs 已接线 |
| [06 运动模型](06_motion_models.md) | **优先**：仓库里真正有的实现 |
| [07 综述](07_survey.md) | 能力边界 |

### 2. 动手时记住

1. 别把综述里的学习型方法当成已实现。  
2. 运动模型在 **common**，不在 `prediction/` 里堆算法。  
3. detection → track → predict **整链未闭合**；找不到预测话题是正常现象。

### 3. 常见坑

| 现象 | 先查 |
|------|------|
| 找不到预测话题 | 流水线未接，属预期 |
| 文档里的 CTRV / 匈牙利 | 未实现，勿对着写业务 |
| 规划要不要等预测 | 不必；用 costmap |
