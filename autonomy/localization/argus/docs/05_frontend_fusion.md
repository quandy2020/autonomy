# 前端与融合

涵盖：预处理 / 同步、动态过滤、退化感知、置信度门控、强度辅助。公式见 [04_math.md](04_math.md)。

## 1. 前端管线

| 组件 | 路径 | 职责 |
|------|------|------|
| `Preprocess` | `frontend/preprocess.*` | 点云格式、盲区、时间戳字段 |
| `SensorSync` | `frontend/sensor_sync.*` | LiDAR / IMU / Image 时间对齐 → `MeasureGroup` |
| `ImuProcess` | `frontend/imu_process.*` | 传播、初始化重力偏置、去畸变 |
| `DynamicFilter*` | `frontend/*_dynamic_filter.*` | 残差前剔除动态点 / 特征 |

集成点（`LivoOdometry`）：

1. `AddImage` → 语义 mask 刷新（若启用）。
2. `ProcessLio`：下采样后、`EsikfStateFilter::Update` 前 `ApplyLidarMask`。
3. `last_lidar_dynamic_ratio()` → ConfidenceGating。

## 2. 动态过滤

**原则：** 在残差构造之前完成；ESIKF 不感知动态逻辑。

| 策略 | 优点 | 缺点 |
|------|------|------|
| `spatiotemporal` | 无训练依赖 | 新区域初始化弱 |
| `semantic` | 召回高 | 需 ONNX 或 mask |
| `none` | 默认 | 不剔除 |

配置：`frontend.dynamic_filter.lidar` / `.visual`；细节 `config/robustness/dynamic_filter.yaml`。

### 2.1 语义数据流

```text
Image → FilterVisual (ONNX / mask_file) → keep_mask HxW
LiDAR body → Rcl,Pcl → 投影 (u,v) → sample mask → FilterLidar
→ ApplyLidarMask → 点面残差
```

| backend | 行为 |
|---------|------|
| `onnx` | 需 `ARGUS_HAS_ONNXRUNTIME` + `semantic_model`；logits / 概率 / 类别图 |
| `mask_file` | `semantic/mask_path` 或 `SetKeepMask`；KITTI `masks/*.pgm` |

模型失败 → pass-through。CMake：`-DARGUS_USE_ONNXRUNTIME=ON`。

### 2.2 KITTI 验证

```bash
slice_kitti --src $KITTI/sequences/00 --dst testdata/kitti_real --frames 3
verify_kitti testdata/kitti_real
offline_kitti --kitti testdata/kitti_real --filter-only
```

## 3. 退化感知

路径：`perception/degeneracy_detector.*`。

第 0 次 ESIKF 迭代：`FIM = H_T_H.block<3,3>(3,3)` → `Analyze` → 可选写入 `options.degeneracy_result`（`last_degeneracy()`）。

配置：`config/robustness/degeneracy.yaml`

| 键 | 含义 |
|----|------|
| `eigenvalue_threshold` | \(\lambda_{\min}\) 过小 → 退化 |
| `condition_number_threshold` | 条件数过大 → 病态 |
| `enable_anisotropic_inflation` | 弱方向 rank-1 膨胀 \(\mathbf{P}\) |
| `inflation_scale` | 膨胀系数 \(\alpha\) |

默认 `enable_degeneracy: true`。退化亦参与 `IsTrackingLost`（重定位触发）。

## 4. ConfidenceGating

路径：`fusion/confidence_gating.*`、`anisotropic_covariance.*`。

```cpp
ObservationConfidence Evaluate(
    const DegeneracyResult& degeneracy,
    const SensorHealth& health,
    double dynamic_ratio_lidar,
    double dynamic_ratio_visual) const;
```

| 信号 | 动作 |
|------|------|
| LiDAR 弱方向 | 降权 / rank-1 膨胀；非默认整帧关断 |
| 视觉动态比例高 | 降低 `visual_weight` |
| 传感器超时 | `*_valid=false`，跳过该模态 |
| 退化过强 | \(1/\kappa <\) `min_inv_condition` 时跳过 LiDAR |

配置：`config/robustness/confidence_gating.yaml`（`min_lidar_weight` 等）。  
开关：`enable_confidence_gating`（默认 **false**）。  
健康度：`SensorHealthMonitor` 在 `Add*` 打点。

接线顺序（估计线程内）：

```text
残差 → HᵀR⁻¹H → Degeneracy → Gating
  → !lidar_valid → 跳过 LiDAR 更新
  → 否则 H/HTz *= lidar_weight；可选膨胀 P
  → 迭代 ESIKF
```

## 5. 强度辅助

路径：`fusion/intensity_assist.*`。几何退化时，用点云强度纹理估计是否缓解协方差膨胀（乘 `inflation_multiplier`）。

默认 `intensity_aided.enable: true`；与退化联动，见 ESIKF 第 0 次迭代分支。

## 6. 测试

| 用例 | 覆盖 |
|------|------|
| `test_dynamic_filter` | 时空法向 |
| `test_semantic_dynamic_filter` | 投影 + SetKeepMask |
| `test_degeneracy_detector` | FIM 阈值 |
| `test_confidence_gating` | 权重与跳过 |
| `test_intensity_assist` | 强度因子 |
