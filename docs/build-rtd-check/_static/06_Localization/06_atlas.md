# 6. Atlas 视觉 SLAM

Atlas 是 `autonomy/localization` 的核心实现，架构源自 **stella_vslam**（ORB-SLAM 系列），提供单目 / 双目 / RGB-D 视觉 SLAM 与纯定位能力。

> 数学推导见 [§3 数学原理](03_math.md)；架构见 [§5 模块架构](05_architecture.md)。

---

## 6.1 系统入口 `atlas::system`

### 6.1.1 构造与启动

```cpp
system(const std::shared_ptr<config>& cfg, const std::string& vocab_file_path);
void startup(const bool need_initialize = true);
void shutdown();
```

构造时完成：

1. 解析 YAML → 创建 `camera::base`（工厂 `camera_factory`）
2. 加载 ORB 词袋 → `bow_vocabulary` + `bow_database`
3. 初始化 `map_database`、`orb_params_database`
4. 创建三模块与 ORB extractor（左/右/初始化三套）
5. 选择 `map_database_io`（msgpack / sqlite3）

`startup()` 启动 `mapping_thread_` 与 `global_optimization_thread_`。

### 6.1.2 数据输入 API

| 方法 | 输入 | 返回 |
|------|------|------|
| `feed_monocular_frame(img, t)` | CV_8UC1/UC3 | `shared_ptr<Mat44_t>` (T_cw) |
| `feed_stereo_frame(L, R, t)` | 已校正立体对 | 同上 |
| `feed_RGBD_frame(rgb, depth, t)` | 对齐 RGB-D | 同上 |
| `create_*_frame(...)` | 预构造 frame | 供自定义 pipeline |

内部流程：`create_*_frame` → ORB 提取 → `feed_frame` → `tracking_module::feed_frame`。

---

## 6.2 相机模型 `camera/`

### 6.2.1 类层次

```
camera::base (抽象)
├── perspective      # 针孔 + Brown 畸变
├── fisheye          # Kannala-Brandt 鱼眼
├── equirectangular  # 等距圆柱全景
└── radial_division  # 径向分割模型 (Fitzgibbon)
```

### 6.2.2 核心虚函数

| 方法 | 作用 |
|------|------|
| `convert_keypoint_to_bearing` | 像素 → 单位 bearing |
| `reproject_to_image` | 3D 相机坐标 → 像素 |
| `undistort_keypoints` | 畸变校正 |
| `triangulate_stereo` | 立体三角化（双目） |
| `depth_to_bearing` | 深度图 → bearing（RGB-D） |

### 6.2.3 透视模型内参

```cpp
// camera::perspective
Mat33_t eigen_cam_matrix_ << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1;
Vec5_t eigen_dist_params_ << k1_, k2_, p1_, p2_, k3_;
```

---

## 6.3 特征提取 `feature/`

### 6.3.1 ORB Extractor

`feature::orb_extractor` 封装 OpenCV ORB 改进版：

- 多尺度金字塔（`num_levels_`, `scale_factor_`）
- 网格划分加速均匀分布
- 可选 mask 排除无效区域

### 6.3.2 ORB 参数

```yaml
Feature:
  scale_factor: 1.2
  num_levels: 8
  ini_fast_threshold: 20
  min_fast_threshold: 7
```

预计算量（`orb_params`）：

- `scale_factors_[ℓ] = 1.2^ℓ`
- `inv_level_sigma_sq_[ℓ] = 1 / (1.2^{2ℓ})`（用于 g2o 信息矩阵）

---

## 6.4 数据结构 `data/`

### 6.4.1 frame

单帧观测容器：

- ORB 关键点 + 描述子
- 2D–3D 关联（`landmarks_`）
- 临时位姿估计
- BoW 向量（关键帧化时使用）

### 6.4.2 keyframe

从 frame 升级，持久化存入 `map_database`：

- 唯一 `id_`、时间戳 `timestamp_`
- 位姿 `pose_cw_`（可优化）
- `frame_observation`（特征观测）
- 共视图 `graph_node`（spanning tree + 共视边）
- BoW 向量（回环检索）

### 6.4.3 landmark

3D 路标：

- 世界坐标 `pos_w_`
- 观测列表 `observations_`（keyframe → 特征索引）
- 参考关键帧 `ref_keyfrm_`
- 可见/找到计数（质量评估）

### 6.4.4 map_database

Central 存储，管理所有 keyframe / landmark 的 CRUD、共视图、最大 ID 分配。

---

## 6.5 跟踪 `tracking_module`

### 6.5.1 状态机

```cpp
enum class tracker_state_t { Initializing, Tracking, Lost };
```

### 6.5.2 初始化 `module/initializer`

单目流程（`initialize::perspective`）：

1. 第一帧存为参考帧
2. 第二帧与参考帧匹配
3. 并行估计 $\mathbf{E}$（5pt RANSAC）与 $\mathbf{H}$（DLT RANSAC）
4. 选内点更多者恢复 $R, t$
5. 三角化 → 创建初始路标与两个关键帧

阈值（YAML `Initializer`）：

```yaml
Initializer:
  min_num_triangulated_pts: 100
```

### 6.5.3 正常跟踪

1. **运动模型预测**：$\hat{T} = \Delta T \cdot T_{last}$
2. **帧间匹配**：`frame_tracker`（描述子匹配 + 旋转直方图）
3. **局部地图更新**：选取共视关键帧的路标
4. **投影搜索**：`match/projection` 在预测位姿附近搜索
5. **PnP + g2o 优化**：`pose_optimizer_g2o`
6. **关键帧决策**：`keyframe_inserter`（视差、跟踪点数、时间间隔）

### 6.5.4 重定位 `module/relocalizer`

丢失时：

1. BoW 检索候选关键帧（`bow_tree`）
2. 特征匹配 + PnP RANSAC
3. 成功则恢复 `Tracking`

用户可调用 `relocalize_by_pose(T_cw)` 注入先验位姿，在附近关键帧搜索。

---

## 6.6 建图 `mapping_module`

### 6.6.1 新关键帧处理流水线

| 步骤 | 函数 | 说明 |
|------|------|------|
| 1 | `store_new_keyframe` | 写入 map_database |
| 2 | `create_new_landmarks` | 与共视帧 epipolar 匹配 + 三角化 |
| 3 | `fuse_landmark_duplication` | 融合重复路标 |
| 4 | `local_bundle_adjuster` | g2o LBA |
| 5 | `local_map_cleaner` | 剔除冗余关键帧/路标 |

### 6.6.2 三角化条件

- 基线 $> \max(\tau_b, \eta \cdot d_{\text{median}})$
- 视差角 > 阈值
- 重投影误差 < `residual_rad_thr_`（0.2°）
- 深度为正（cheirality）

### 6.6.3 局部 BA

优化窗口：当前关键帧 + 共视关键帧（最多 10 个用于路标生成/融合）。

可被 `abort_local_BA()` 中断（Tracking 需快速响应时）。

---

## 6.7 全局优化 `global_optimization_module`

### 6.7.1 回环检测 `module/loop_detector`

1. 将关键帧 BoW 向量插入 `bow_database`
2. 查询相似候选（排除共视帧）
3. 几何验证：Sim3 / 本质矩阵 + 匹配数
4. `transform_optimizer` 精化 Sim3

### 6.7.2 回环校正

`correct_loop()` 步骤：

1. `get_Sim3s_before/after_loop_correction`
2. `correct_covisibility_landmarks` — 路标坐标变换
3. `correct_covisibility_keyframes` — 关键帧位姿变换
4. `replace_duplicated_landmarks` — 合并重复路标
5. `extract_new_connections` — 更新共视图
6. 异步 `loop_bundle_adjuster` — 全局 BA

### 6.7.3 手动回环

```cpp
system.request_loop_closure(keyfrm1_id, keyfrm2_id);
```

---

## 6.8 优化后端 `optimize/`

### 6.8.1 g2o 顶点与边

| 类型 | 文件 | 说明 |
|------|------|------|
| SE3 位姿顶点 | `internal/se3/*` | 关键帧/帧位姿 |
| 路标顶点 | `internal/landmark_vertex` | 3D 点（XYZ） |
| 重投影边 | `reproj_edge_wrapper` | bearing 误差 |
| Sim3 边 | `internal/sim3/*` | 回环约束 |
| 先验边 | `perspective_pose_opt_edge` | 固定参考 |

### 6.8.2 工厂模式

```cpp
local_bundle_adjuster_factory  // 按 YAML backend 创建 g2o 实现
pose_optimizer_g2o             // 单帧位姿优化
graph_optimizer                // 全局图优化
```

---

## 6.9 求解器 `solve/`

| 求解器 | 最小样本 | RANSAC | 用途 |
|--------|----------|--------|------|
| `essential_solver` | 5 / 8 | ✓ | 初始化 |
| `homography_solver` | 4 | ✓ | 平面初始化 |
| `fundamental_solver` | 8 | ✓ | 基础矩阵 |
| `pnp_solver` | 4 | ✓ | 跟踪/重定位 |

---

## 6.10 地图 I/O `io/`

| 格式 | 类 | 特点 |
|------|-----|------|
| msgpack | `map_database_io_msgpack` | 紧凑、快速 |
| sqlite3 | `map_database_io_sqlite3` | 结构化、可查询 |

轨迹导出（`trajectory_io`）：

- 格式：`TUM`、`KITTI`、`EUROC`
- API：`save_frame_trajectory` / `save_keyframe_trajectory`

---

## 6.11 示例配置

| 数据集 | 配置文件 | 模式 |
|--------|----------|------|
| EuRoC | `example/euroc/EuRoC_stereo.yaml` | 双目 |
| KITTI | `example/kitti/KITTI_stereo_*.yaml` | 双目 |
| TUM RGB-D | `example/tum_rgbd/TUM_RGBD_rgbd_1.yaml` | RGB-D |
| TUM VI | `example/tum_vi/TUM_VI_mono.yaml` | 单目 |
| AIST | `example/aist/fisheye.yaml` | 鱼眼 |

---

## 6.12 Marker 支持（可选）

Atlas 支持 ArUco/AprilTag 类 marker（`marker_model/`、`data/marker*`）：

- `module/marker_initializer` — marker 辅助初始化
- g2o marker 顶点 — 已知 3D 位置的约束

适用于纹理匮乏环境的辅助定位。

---

## 6.13 性能与资源

| 组件 | 典型耗时 | 内存 |
|------|----------|------|
| ORB 提取 | 5–15 ms | 与分辨率成正比 |
| 跟踪 + PnP | 2–5 ms | — |
| LBA | 20–100 ms | 局部窗口 |
| Loop BA | 1–30 s | 全图（异步） |
| 词袋索引 | < 1 ms | 加载时一次性 |

**建议**：Loop BA 运行时 Tracking 仍可继续；若卡顿可 `abort_loop_BA()`。

---

## 6.14 与 ORB-SLAM3 对比

| 特性 | Atlas | ORB-SLAM3 |
|------|-------|-----------|
| 传感器 | 单目/双目/RGB-D | + IMU（VI-SLAM） |
| 多地图 | 单地图 | Atlas 多地图 |
| 优化后端 | g2o | g2o / DBoW2 |
| 鱼眼/全景 | ✓ | 部分 |
| 许可证 | Apache 2.0 | GPL-3.0 |

Autonomy Atlas 更适合 Apache 生态集成；需 IMU 融合时可扩展 `initialize` 与预积分模块。
