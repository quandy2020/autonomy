# 8. 行为与交互预测

本文描述考虑**自车动作**与**障碍交互**的高阶预测方法，供未来 Prediction 模块扩展参考。

---

## 8.1 行为预测 vs 轨迹预测

| 维度 | 轨迹预测 | 行为预测 |
|------|----------|----------|
| 输出 | 位置序列 $\hat{\mathbf{x}}(t)$ | 意图/机动标签 + 轨迹 |
| 方法 | CV/CTRV/卡尔曼 | 规则、学习、博弈 |
| 不确定性 | 高斯椭圆 | 离散意图 + 连续轨迹 |

---

## 8.2 规则型行为预测

### 8.2.1 车道约束

若地图提供车道线，障碍轨迹投影到车道中心线：

$$
\hat{\mathbf{p}}(\tau) = \mathrm{ProjectOnLane}\big(\mathbf{p}_0 + \mathbf{v} \tau, \mathcal{L}\big)
$$

### 8.2.2 交叉口意图

| 观测 | 推断意图 | 轨迹模式 |
|------|----------|----------|
| 接近交叉口 + 转向灯左 | 左转 | CTRV $\omega > 0$ |
| 直行车道 + 匀速 | 直行 | CV |
| 接近停车线 + 减速 | 停车 | CA $a < 0$ |

---

## 8.3 交互式预测

### 8.3.1 问题形式化

自车策略 $\pi_{ego}$，障碍 $i$ 策略 $\pi_i$，联合预测：

$$
\hat{\mathcal{T}}_i = f\big(\mathbf{x}_i, \pi_{ego}, \pi_i, \mathcal{M}\big)
$$

其中 $\mathcal{M}$ 为地图/场景上下文。

### 8.3.2 反应式模型

障碍对自车动作的响应（简化）：

$$
\mathbf{v}_i^{react} = \mathbf{v}_i^{nom} + K_{react} \cdot \mathrm{repulse}(\mathbf{p}_{ego}, \mathbf{p}_i)
$$

### 8.3.3 博弈论方法

Stackelberg 博弈：自车为 leader，障碍为 follower，求均衡轨迹。

**工程简化**：枚举自车候选路径 $\{\mathcal{P}_{ego}^{(m)}\}$，对每条路径预测障碍响应轨迹。

---

## 8.4 学习型行为预测

### 8.4.1 代表方法

| 方法 | 特点 |
|------|------|
| Social LSTM | 社交池化，行人轨迹 |
| VectorNet | 矢量化地图 + 轨迹，Waymo 开源 |
| LaneGCN | 图卷积车道拓扑 |
| MTR | Transformer 多模态 |

### 8.4.2 输入特征

- 障碍历史轨迹（$T_h$ 秒）
- 地图：车道、交叉口、人行横道
- 自车状态与规划路径
- 其他障碍状态（社交）

### 8.4.3 输出

$K$ 条轨迹 + 概率 $\{\hat{\mathcal{T}}^{(k)}, \pi_k\}_{k=1}^K$。

---

## 8.5 碰撞风险与 TTC

### 8.5.1 Time To Collision

$$
\mathrm{TTC} = \min_{\tau > 0} \left\{ \tau : \|\mathbf{p}_{ego}(\tau) - \mathbf{p}_i(\tau)\| < r_{safe} \right\}
$$

### 8.5.2 风险评分

$$
\mathrm{risk}_i = \frac{1}{\mathrm{TTC}_i + \epsilon} \cdot \mathbf{1}_{\{\mathrm{TTC}_i < T_{\mathrm{thresh}}\}}
$$

供 Planning 做速度调整或换道决策。

---

## 8.6 Autonomy 扩展建议

| 阶段 | 能力 | 实现路径 |
|------|------|----------|
| 近期 | CV/CTRV 轨迹 | motion_model + 卡尔曼 |
| 中期 | 车道约束规则 | 依赖高精地图 |
| 远期 | VectorNet ONNX | common/network 推理 |

---

## 8.7 与 Control 的 TEB 调研

`docs/source/09_Control/09_survey.md` 提及 TEB（Timed Elastic Band）可结合 prediction 做动态避障。TEB 在代价函数中加入动态障碍项：

$$
J_{dyn} = \sum_k \sum_i w_i \cdot \mathrm{penalty}\big(\mathbf{p}_k, \hat{\mathbf{p}}_i(t_k)\big)
$$

此为规划/控制层消费预测的典型方式。
