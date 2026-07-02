(localization-usage)=
# 4. 使用指南

本章按 **Atlas 视觉 SLAM** 与 **AMCL 配置（待集成）** 分节说明。

---

## 4.1 Atlas 使用

### 4.1.1 配置入口

| 项 | 值 |
|----|-----|
| 配置文件 | YAML（见 `atlas/example/` 各数据集模板） |
| 加载 API | `atlas::config(config_file_path)` |
| 词袋文件 | ORB BoW vocabulary（`.fbow`） |
| 系统入口 | `atlas::system` |

### 4.1.2 YAML 配置结构

```yaml
Camera:           # 相机内参、畸变、setup/model 类型
Preprocessing:    # 图像预处理（min_size 等）
Feature:          # ORB 金字塔参数
Mapping:          # 局部建图 / LBA 后端
Tracking:         # 跟踪 / 位姿优化后端
KeyframeInserter: # 关键帧插入策略
Relocalizer:      # 重定位参数
LoopDetector:     # 回环检测
Initializer:      # 初始化阈值
System:           # map_format, 匹配网格尺寸
```

### 4.1.3 推荐使用路径

| 路径 | 适用 | 你需要做的 |
|------|------|------------|
| **A. SLAM 建图** | 离线/在线建图 | `startup()` + 持续 `feed_*` + `save_map_database` |
| **B. 纯定位** | 已有地图 | `load_map_database` + `disable_mapping_module` |
| **C. 数据集评测** | EuRoC/KITTI/TUM | 使用 `example/` 下对应 YAML |

### 4.1.4 端到端集成（路径 A）

```
启动顺序:
  1. 加载 YAML + 词袋
  2. system::startup()
  3. 相机驱动回调 → feed_monocular/stereo/RGBD_frame
  4. 获取 pose (T_cw) → 转换到 base_link → 发布 TF
  5. 可选: map_publisher 可视化稀疏地图
  6. shutdown() 前 save_map_database / save_keyframe_trajectory
```

### 4.1.5 API 与调用时机

| API | 谁调用 | 何时 |
|-----|--------|------|
| `startup()` | 系统启动 | 配置与词袋加载完成后 |
| `feed_*_frame()` | 相机回调 | 每帧图像到达 |
| `load_map_database()` | 纯定位模式 | `startup(false)` 之后 |
| `relocalize_by_pose()` | 外部先验 | 跟踪丢失或 GPS/IMU 辅助 |
| `enable_loop_detector()` | 建图模式 | 大场景需回环时 |
| `save_map_database()` | 退出前 | 持久化地图 |
| `shutdown()` | 系统关闭 | 停止所有线程 |

### 4.1.6 相机模型选型

| `model` | 适用镜头 | 畸变参数 |
|---------|----------|----------|
| `perspective` | 普通针孔 | k1, k2, p1, p2, k3 |
| `fisheye` | 广角鱼眼 | k1–k4 |
| `equirectangular` | 360° 全景 | 无 |
| `RadialDivision` | 轻畸变 | k1, k2 |

| `setup` | 输入 | 注意 |
|---------|------|------|
| `monocular` | 单目图像 | 尺度不可观，需运动 |
| `stereo` | 左右校正图 | 必须 stereo-rectify |
| `rgbd` | RGB + 深度 | 必须对齐，`depthmap_factor_` 正确 |

### 4.1.7 关键参数调优

**Feature**

| 参数 | 默认 | 说明 |
|------|------|------|
| `scale_factor` | 1.2 | 金字塔尺度比 |
| `num_levels` | 8 | 金字塔层数 |
| `ini_fast_threshold` | 20 | 初始化 FAST 阈值 |
| `min_fast_threshold` | 7 | 跟踪 FAST 阈值 |

**Tracking**

| 参数 | 默认 | 说明 |
|------|------|------|
| `max_num_local_keyfrms` | 60 | 局部地图关键帧上限 |
| `enable_auto_relocalization` | true | 丢失自动重定位 |
| `reloc_distance_threshold` | 0.2 m | 按位姿重定位距离阈值 |
| `margin_local_map_projection` | 5.0 px | 投影匹配搜索半径 |

**Mapping**

| 参数 | 默认 | 说明 |
|------|------|------|
| `baseline_dist_thr_ratio` | 0.02 | 三角化基线/深度比 |
| `num_temporal_keyframes` | 15 | 时序关键帧数量 |
| `backend` | g2o | 优化后端 |

**LoopDetector**

| 参数 | 说明 |
|------|------|
| `backend` | g2o |
| `fix_scale_in_Sim3_estimation` | 单目 true，双目/RGB-D false |

### 4.1.8 地图持久化

| 格式 | API | 说明 |
|------|-----|------|
| msgpack | `map_format: "msgpack"` | 默认，紧凑二进制 |
| sqlite3 | `map_format: "sqlite3"` | 关系型，便于查询 |

```cpp
slam.save_map_database("output/map.msgpack");
slam.load_map_database("output/map.msgpack");
slam.save_keyframe_trajectory("traj.txt", "TUM");  // 或 "KITTI", "EUROC"
```

---

## 4.2 AMCL 配置（待集成）

### 4.2.1 配置入口

| 项 | 值 |
|----|-----|
| 配置文件 | `config/localization/localization.lua` |
| 子配置 | `config/localization/amcl/amcl.lua` |
| Proto | `autonomy.localization.proto.AmclOptions`（规划中） |

### 4.2.2 启用方式（规划）

```lua
-- config/autonomy.lua
include "localization/localization.lua"

AUTONOMY = {
    localization = AUTONOMY_LOCALIZATION,
}
```

### 4.2.3 依赖话题

| 话题 | 类型 | 方向 |
|------|------|------|
| `/map` | OccupancyGrid | 订阅 |
| `/scan` | LaserScan | 订阅 |
| `/odom` | Odometry | 订阅（可选，用于运动模型） |
| TF `map→odom` | Transform | 发布 |

---

## 4.3 Cartographer 配置（待集成）

配置模板位于 `config/localization/cartographer/`：

| 文件 | 内容 |
|------|------|
| `map_builder.lua` | 2D/3D map builder |
| `trajectory_builder_2d.lua` | 2D 前端 |
| `trajectory_builder_3d.lua` | 3D 前端 |
| `pose_graph.lua` | 位姿图优化 |

---

## 4.4 与导航栈集成

```
┌─────────────┐     ┌──────────────┐     ┌─────────────┐
│   Driver    │────►│ Localization │────►│  Transform  │
│ camera/scan │     │ Atlas / AMCL │     │  map↔base   │
└─────────────┘     └──────┬───────┘     └──────┬──────┘
                           │                     │
                           ▼                     ▼
                    ┌─────────────┐       ┌─────────────┐
                    │ Map (可选)   │       │  Planning   │
                    └─────────────┘       └─────────────┘
```

| 集成点 | Atlas | AMCL |
|--------|-------|------|
| TF 输出 | 需桥接 `T_cw` → `map→odom` | 原生支持 |
| 静态地图 | 可导出稀疏地图 | 消费 `/map` |
| 传感器 | 相机 | 激光 + 里程计 |

---

## 4.5 算法选型

| 场景 | 推荐 | 理由 |
|------|------|------|
| 视觉机器人 / 无激光 | Atlas | 已实现，支持单目/双目/RGB-D |
| 已知 2D 地图 + 激光 | AMCL | 成熟、低算力（待集成） |
| 未知环境激光 SLAM | Cartographer | 2D/3D 激光（待集成） |
| 大场景视觉 + 回环 | Atlas + loop | BoW 回环 + GBA |
| 短走廊 / 纹理弱 | 双目/RGB-D Atlas | 深度可观，鲁棒性更好 |

---

## 4.6 性能参考

| 模式 | 典型帧率 | 主要瓶颈 |
|------|----------|----------|
| 单目 640×480 | 20–30 Hz | ORB 提取 + 匹配 |
| 双目 752×480 | 15–25 Hz | 立体匹配 + LBA |
| RGB-D 640×480 | 20–30 Hz | 深度处理 + LBA |
| Loop BA | 异步 | g2o GBA，可能数秒 |

---

## 4.7 故障排查

| 现象 | 可能原因 | 排查 |
|------|----------|------|
| 长期 Initializing | 纹理不足 / 运动不够 | 增加纹理、做充分平移 |
| 频繁 Lost | 快速运动 / 曝光变化 | 降低速度、提高 `min_fast_threshold` |
| 尺度漂移（单目） | 无回环 | `enable_loop_detector()` |
| 双目不三角化 | 未校正 / baseline 错 | 检查 `focal_x_baseline` |
| RGB-D 深度异常 | `depthmap_factor_` 错误 | 对齐 TUM/Kinect 标准 |
| 地图加载失败 | 格式不匹配 | 检查 `map_format` |
| AMCL 不工作 | 尚未集成 | 使用 Atlas 或等待集成 |

---

## 4.8 调试工具

| 工具 | 用途 |
|------|------|
| `frame_publisher` | 当前帧特征、跟踪状态 |
| `map_publisher` | 稀疏 3D 地图与关键帧 |
| `save_frame_trajectory` | 逐帧轨迹导出 |
| `print_info()` | 打印系统与相机参数 |

```cpp
auto frame_pub = slam.get_frame_publisher();
auto map_pub = slam.get_map_publisher();
slam.print_info();
```
