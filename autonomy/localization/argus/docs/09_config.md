# 配置

## 1. 文件布局

```text
config/
├── livo_esikf.yaml              # 主配置
├── loop_closure.yaml
├── calibration/                 # 内外参（与算法开关分离）
│   ├── camera_pinhole.yaml
│   ├── extrinsic_lidar_camera.yaml   # Rcl / Pcl
│   └── extrinsic_imu_lidar.yaml      # ext_R / ext_T
└── robustness/
    ├── degeneracy.yaml
    ├── dynamic_filter.yaml
    ├── confidence_gating.yaml
    └── map_cleaning.yaml
```

LIO / LO：复制主配置改 `odometry.modalities`。完整 LIVO 需 `camera.calib_file` + `vio.*`。

## 2. 主配置段

| 段 | 含义 |
|----|------|
| `odometry.type` / `modalities` | livo / lio / lo；含 `camera` 时启用 VIO |
| `camera.calib_file` | 内参 → `VIOManager::InitializeVIO` |
| `extrinsic.lidar_camera` | LiDAR→相机 |
| `extrinsic.imu_lidar` | LiDAR→IMU |
| `vio.*` | grid / patch / 迭代 / 曝光 |
| `backend.filter` | `esikf`；`max_iterations` |
| `frontend.dynamic_filter` | `spatiotemporal` / `semantic` / `none` |
| `robustness.*` | 退化 / 门控 / 强度 |
| `map.self_cleaning` | 自清洁 |
| `loop_closure` / `cross_session` | 异步回环 |
| `relocalization.*` | BnB 池、触发、先验路径 |
| `mapping.enable` / `output_dir` | 在线 MapBuilder |
| `threading.backend_threads` | **必须为 1** |

嵌套文件相对主 YAML 目录解析。行尾 `#` 注释勿写在 OpenCV `FileStorage` 数值同行（会吞值）。

## 3. 加载

```cpp
auto r = argus::odometry::LoadLivoOdometryConfig("config/livo_esikf.yaml");
// r.config → LivoOdometry / offline_* / livo_esikf_node
```

实现：`argus/odometry/load_livo_config.*`（OpenCV `FileStorage`）。

## 4. 校验（加载失败即拒绝）

| 规则 | 动作 |
|------|------|
| `backend_threads != 1` | 失败 |
| `loop_closure.enable` 且 `async: false` | 失败 |
| `semantic` + `backend=onnx` 且空 `semantic_model` | 失败 |
| 无 LiDAR 的 ONLY_VIO | 失败 |
| `camera` 模态且 `calib_file` 不存在 | 失败 |
| `relocalization.trigger` 非法枚举 | 失败 |

推荐生产组合：退化 / 强度 / mapping / reloc 默认 on；门控 / 回环 / 自清洁按场景打开。见 [12_production.md](12_production.md)。

## 5. 标定文件示例键

**相机**（`camera_pinhole.yaml`）：模型类型、`fx fy cx cy`、畸变、分辨率。  
**LiDAR→相机**：`Rcl`（3×3）、`Pcl`（3）。  
**LiDAR→IMU**：`ext_R`、`ext_T`。

Oxford Spires 等距相机：`scripts/convert_kalibr_camera.py` → `camera_equidistant.yaml`。
