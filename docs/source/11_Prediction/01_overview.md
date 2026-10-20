(prediction-overview)=
# 1. 模块概览

第一次接触 `autonomy/prediction/`，需要知道能指望什么、不能指望什么。

说明动态障碍「预测」在 Autonomy 里的**目标形态**与**当前缺口**。

理想链路：感知给出障碍状态 → 预测 extrapolate → 规划避障。  
**当前未实现。** `PredictionServer` 无方法体；主栈启动路径里**没有**构造它。车上导航仍：**传感器 → costmap → Planning → Control**。

可直接用的是 **`common/motion_model`** 与 **卡尔曼** 代码，供实验或自研节点调用（[§6](06_motion_models.md)）。

- `autonomy/prediction/` ≠ 完整预测产品；别在里头找 Predictor 插件目录。  
- Perception base 的 tracks **不会**自动进 prediction。  
- 文档中的 CTRV、交互式预测等多为数学/规划描述，**无**对应 C++ 服务。

| 需求 | 入口 |
|------|------|
| 写前向传播 | [§6 运动模型](06_motion_models.md) |
| 看 Server 缺什么 | [§5 架构](05_architecture.md) |
| 联调车上功能 | 回到 [10 Perception](../10_Perception/08_obstacle_perception.md) costmap |

### 常见坑

| 现象 | 说明 |
|------|------|
| 找预测话题 | 正常不存在 |
| 启用 prediction.pb.txt 无效果 | 无消费者 |

### 延伸阅读

[§7 综述](07_survey.md)
