# 9. 感知算法综述（Survey）

本文从**学术发展史、算法体系、工程实践**三个维度，系统综述移动机器人环境感知领域，并明确 Autonomy `perception` 模块的定位与选型依据。

> 公式推导见 [03_math.md](03_math.md)；实现细节见 [05_architecture.md](05_architecture.md) 与各子文档。

---

## 9.1 概述：感知在导航栈中的位置

| 子问题 | 英文 | 典型模块 | 频率 |
|--------|------|----------|------|
| **环境感知** | **Perception** | **`perception`** | **10–30 Hz** |
| 定位 | Localization | `localization` | 10–50 Hz |
| 预测 | Prediction | `prediction` | 10–20 Hz |
| 路径规划 | Planning | `planning` | 1–5 Hz |
| 运动控制 | Control | `control` | 10–50 Hz |

**感知**将原始传感器数据转化为结构化环境表示，是自动驾驶与移动机器人**理解世界**的基础层。

```
Sensors ──→ Perception ──→ Prediction ──→ Planning ──→ Control
                │
                └──→ Map (costmap / semantic map)
```

---

## 9.2 感知任务分类

### 9.2.1 按传感器模态

| 模态 | 典型任务 | 代表算法 |
|------|----------|----------|
| 2D 激光 | 障碍检测、SLAM | Hector SLAM, GMapping |
| 3D 点云 | 3D 检测、地面分割 | PointPillars, Patchwork |
| RGB 相机 | 2D 检测、语义分割 | YOLO, Mask R-CNN |
| RGB-D | 3D 检测、场景理解 | VoxelNet, Frustum PointNet |
| 多模态融合 | 鲁棒检测 | BEVFusion, MV3D |

### 9.2.2 按输出类型

| 输出 | 用途 | Autonomy 对应 |
|------|------|---------------|
| 几何障碍 | 避障、costmap | `ObstacleLayer` ✅ |
| 2D/3D 检测框 | 动态障碍跟踪 | `vision_msgs` ⏳ |
| 语义分割 | 可行驶区域、规则推理 | 规划 |
| 占据栅格 | 全局规划 | `map` 静态层 |

---

## 9.3 2D 激光感知

### 9.3.1 射线模型

激光第 $i$ 束：$p_i = o + r_i \hat{d}_i$。障碍点直接写入 costmap 或经聚类后输出障碍列表。

### 9.3.2 聚类与跟踪

- **欧氏聚类**：$\|p_i - p_j\| < r$ 归为同一障碍
- **跟踪**：卡尔曼滤波 + 数据关联（匈牙利算法）

Autonomy 当前通过 `ObstacleLayer` 实现射线级障碍，无显式聚类。

---

## 9.4 视觉目标检测

### 9.4.1 发展时间轴

<div class="planning-timeline-v2">

  <div class="planning-timeline-item">
    <div class="planning-timeline-year">2012</div>
    <div class="planning-timeline-content">
      <strong>AlexNet</strong> — ImageNet 突破，深度学习复兴
    </div>
  </div>

  <div class="planning-timeline-item">
    <div class="planning-timeline-year">2015</div>
    <div class="planning-timeline-content">
      <strong>YOLO v1</strong> — 单阶段实时检测；<strong>Faster R-CNN</strong> — 两阶段精度标杆
    </div>
  </div>

  <div class="planning-timeline-item">
    <div class="planning-timeline-year">2020</div>
    <div class="planning-timeline-content">
      <strong>YOLOv5</strong> — 工程化部署；<strong>DETR</strong> — Transformer 检测
    </div>
  </div>

  <div class="planning-timeline-item">
    <div class="planning-timeline-year">2023+</div>
    <div class="planning-timeline-content">
      <strong>YOLOv8/v11</strong> — ONNX 友好；<strong>RT-DETR</strong> — 实时 Transformer
    </div>
  </div>

</div>

### 9.4.2 算法对比

| 算法 | 类型 | 速度 | 精度 | 部署 |
|------|------|------|------|------|
| YOLOv8 | 单阶段 | ⭐⭐⭐ | ⭐⭐ | ONNX ✅ |
| Faster R-CNN | 两阶段 | ⭐ | ⭐⭐⭐ | 较重 |
| SSD | 单阶段 | ⭐⭐ | ⭐⭐ | 中等 |
| RT-DETR | Transformer | ⭐⭐ | ⭐⭐⭐ | 较新 |

**Autonomy 选型**：`common/network` 针对 YOLO 网格头优化，推荐 YOLOv8 ONNX 作为首个 Perception 插件。

---

## 9.5 3D 点云感知

### 9.5.1 经典流水线

```
点云 → 体素下采样 → 地面分割 → 聚类 → 3D 框拟合
```

### 9.5.2 深度学习方法

| 方法 | 特点 |
|------|------|
| PointPillars | 柱状体素 + 2D CNN，实时性好 |
| VoxelNet | 3D 体素 CNN |
| PointNet++ | 点级特征，精度高但慢 |
| BEVFusion | 多模态 BEV 融合 |

---

## 9.6 多目标跟踪（MOT）

| 组件 | 算法 |
|------|------|
| 运动模型 | CV / CTRV / CA |
| 数据关联 | 匈牙利、JPDA、DeepSORT |
| 状态估计 | 卡尔曼滤波、粒子滤波 |

详见 [03_math.md §3.5](03_math.md#35-卡尔曼滤波跟踪多目标)。

---

## 9.7 工程实践考量

| 因素 | 建议 |
|------|------|
| 实时性 | 感知频率 ≥ 10 Hz；YOLO nano 适合边缘设备 |
| 鲁棒性 | 多传感器融合；激光作 costmap 兜底 |
| 可维护性 | 插件化；配置驱动 |
| 与 Nav2 对齐 | `vision_msgs`、costmap 接口 |

---

## 9.8 Autonomy 能力矩阵

| 能力 | 状态 | 路径 |
|------|------|------|
| 激光 → costmap | ✅ | `map/costmap_2d/ObstacleLayer` |
| 点云 → costmap | ✅ | `VoxelLayer` |
| YOLO ONNX 推理 | ✅ | `common/network` |
| vision_msgs 输出 | ⏳ | 消息已定义，Perception 未发布 |
| PerceptionServer | ⏳ | 骨架 |
| 多目标跟踪 | ❌ | 待实现 |
| 语义分割 | ❌ | 待实现 |

---

## 9.9 参考文献与延伸阅读

| 主题 | 参考 |
|------|------|
| YOLO | [Redmon et al., "You Only Look Once: Unified, Real-Time Object Detection"](https://arxiv.org/abs/1506.02640) |
| PointPillars | [Lang et al., "PointPillars: Fast Encoders for Object Detection from Point Clouds"](https://openaccess.thecvf.com/content_CVPR_2019/html/Lang_PointPillars_Fast_Encoders_for_Object_Detection_From_Point_Clouds_CVPR_2019_paper.html) |
| Apollo Perception | [Baidu Apollo Perception 开源文档](https://apollo.baidu.com/docs/apollo/latest/md_modules_2perception_2README.html) |
| Nav2 Costmap | [Nav2 Costmap 2D](https://docs.nav2.org/) |
| vision_msgs | [ROS 2 `vision_msgs` 包](https://github.com/ros-perception/vision_msgs) |
