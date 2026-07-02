# 9. 预测算法综述（Survey）

本文系统综述移动障碍轨迹预测领域，并明确 Autonomy `prediction` 模块的定位。

> 公式见 [03_math.md](03_math.md)；运动模型见 [06_motion_models.md](06_motion_models.md)。

---

## 9.1 预测在自动驾驶管线中的位置

```
Perception → Prediction → Planning → Control
     │            │
  检测/跟踪    未来轨迹 + 意图
```

| 模块 | 频率 | 时域 |
|------|------|------|
| Perception | 10–30 Hz | 当前 |
| **Prediction** | **10–20 Hz** | **3–8 s** |
| Planning | 1–5 Hz | 全局 + 局部 |
| Control | 10–50 Hz | 0.5–2 s |

---

## 9.2 方法分类

### 9.2.1 按模型类型

| 类型 | 代表 | 优点 | 缺点 |
|------|------|------|------|
| 物理模型 | CV, CTRV, CA | 可解释、实时 | 长期不准 |
| 滤波 | 卡尔曼、粒子 | 不确定性 | 单模态 |
| 学习 | Social LSTM, VectorNet | 多模态、社交 | 数据、算力 |
| 博弈 | Stackelberg | 交互 | 复杂 |

### 9.2.2 按输出形式

| 形式 | 说明 |
|------|------|
| 单轨迹 | 点估计 |
| 多模态 | $K$ 条轨迹 + 概率 |
| 占据栅格 | 时空占用概率 |
| 意图标签 | 左转/直行/停车 + 轨迹 |

---

## 9.3 发展时间轴

<div class="planning-timeline-v2">

  <div class="planning-timeline-item">
    <div class="planning-timeline-year">1960s</div>
    <div class="planning-timeline-content">
      <strong>卡尔曼滤波</strong> — 线性系统最优估计
    </div>
  </div>

  <div class="planning-timeline-item">
    <div class="planning-timeline-year">1990s</div>
    <div class="planning-timeline-content">
      <strong>IMM</strong> — 交互多模型；<strong>粒子滤波</strong> — 非线性
    </div>
  </div>

  <div class="planning-timeline-item">
    <div class="planning-timeline-year">2016</div>
    <div class="planning-timeline-content">
      <strong>Social LSTM</strong> — 社交轨迹预测
    </div>
  </div>

  <div class="planning-timeline-item">
    <div class="planning-timeline-year">2020</div>
    <div class="planning-timeline-content">
      <strong>VectorNet</strong> — Waymo 矢量化预测
    </div>
  </div>

  <div class="planning-timeline-item">
    <div class="planning-timeline-year">2023+</div>
    <div class="planning-timeline-content">
      <strong>MTR, Wayformer</strong> — Transformer 多模态 SOTA
    </div>
  </div>

</div>

---

## 9.4 物理模型详解

| 模型 | 状态维 | 适用 | Autonomy |
|------|--------|------|----------|
| CV | 4 | 行人、短期 | `LinearMotionModel` ✅ |
| CA | 6 | 加减速车辆 | 可扩展 Linear |
| CTRV | 5 | 转弯车辆 | 待实现 |
| DiffDrive | 5 | 移动机器人 | `DifferentialDriveMotionModel` ✅ |
| Stationary | — | 静态 | `StationaryMotionModel` ✅ |

---

## 9.5 学习型方法对比

| 方法 | 输入 | 多模态 | 实时性 |
|------|------|--------|--------|
| Social LSTM | 轨迹 | ✅ | 中 |
| VectorNet | 轨迹+地图 | ✅ | 中 |
| LaneGCN | 图结构 | ✅ | 中 |
| MTR | 轨迹+地图 | ✅ | 较低 |

**部署**：可导出 ONNX，复用 `common/network`。

---

## 9.6 评估指标

| 指标 | 公式/说明 |
|------|-----------|
| ADE | 平均位移误差 $\frac{1}{K}\sum_k \|\hat{\mathbf{p}}_k - \mathbf{p}_k^{gt}\|$ |
| FDE | 终点位移误差 $\|\hat{\mathbf{p}}_K - \mathbf{p}_K^{gt}\|$ |
| MR | Miss Rate，FDE > 阈值的比例 |
| NLL | 负对数似然（概率预测） |

---

## 9.7 Autonomy 能力矩阵

| 能力 | 状态 |
|------|------|
| CV/CA 运动模型 | ✅ `common/motion_model` |
| 卡尔曼滤波 | ✅ `state_estimation` |
| PredictionServer | ⏳ 骨架 |
| CTRV | ❌ |
| 多模态预测 | ❌ |
| 学习型预测 | ❌ |
| prediction_msgs | ❌ |

---

## 9.8 选型建议

| 场景 | 推荐 |
|------|------|
| 室内移动机器人 | CV + 卡尔曼 |
| 室外低速 | CTRV + 规则 |
| 城市道路 | VectorNet 类（远期） |
| 无 Perception | 不启用 Prediction |

---

## 9.9 参考文献

| 主题 | 参考 |
|------|------|
| CTRV | Werling et al., "Optimal Trajectory Generation" |
| Social LSTM | Alahi et al., CVPR 2016 |
| VectorNet | Gao et al., CVPR 2020 |
| Apollo Prediction | Baidu Apollo 文档 |
