# 2. 快速开始

### 2.1 Cartographer 2D SLAM（默认，推荐）

完整说明见 [Cartographer 使用指南](cartographer/guide.md)。

**三步流程：**

1. 转换 bag（Backpack 2D 数据集）：

```bash
python -m autonomy.tools.bag_convert \
  ~/Downloads/b2-2016-04-27-12-31-41.bag \
  -o ./data/records --backpack-2d
```

2. 启动 SLAM：

```bash
export PATH=$PWD/build/bin:$PATH
export AUTOLINK_LAUNCH_PATH=$PWD/autonomy/localization/launch
autolink_launch start cartographer_2d.launch
```

3. 回放 record，观察 `map` 话题与 `data/map.pgm` 输出。

保存地图状态：

```bash
localization \
  --configuration_directory=config/localization/cartographer \
  --configuration_basename=backpack_2d.lua \
  --save_state_filename=data/map.pbstream
```

---

### 2.2 Atlas 视觉 SLAM

1. 准备 **ORB 词袋文件**（`orb_vocab.fbow` 或项目指定格式）与 **相机 YAML 配置**
2. 构造 `atlas::system`，调用 `startup()` 启动三线程 SLAM
3. 按传感器类型循环调用 `feed_monocular_frame` / `feed_stereo_frame` / `feed_RGBD_frame`

### 2.3 最小 C++ 示例（单目）

```cpp
#include "autonomy/localization/atlas/system.hpp"
#include "autonomy/localization/atlas/config.hpp"

// 1. 加载配置
auto cfg = std::make_shared<autonomy::localization::atlas::config>(
    "autonomy/localization/atlas/example/tum_vi/TUM_VI_mono.yaml");

// 2. 构造系统（需 ORB 词袋路径）
autonomy::localization::atlas::system slam(
    cfg, "/path/to/orb_vocab.fbow");

// 3. 启动
slam.startup();

// 4. 逐帧输入
for (const auto& [timestamp, image] : image_stream) {
    auto pose = slam.feed_monocular_frame(image, timestamp);
    if (pose) {
        // pose 为 T_cw (4×4)
    }
}

// 5. 保存轨迹与地图
slam.save_keyframe_trajectory("traj.txt", "TUM");
slam.save_map_database("map.msgpack");
slam.shutdown();
```

### 2.4 双目示例（EuRoC）

配置文件：`autonomy/localization/atlas/example/euroc/EuRoC_stereo.yaml`

```cpp
auto cfg = std::make_shared<atlas::config>(".../EuRoC_stereo.yaml");
atlas::system slam(cfg, vocab_path);
slam.startup();

// 左右图必须已 stereo-rectify
auto pose = slam.feed_stereo_frame(left_img, right_img, timestamp);
```

关键配置项：

```yaml
Camera:
  setup: "stereo"
  model: "perspective"
  fx: 435.20
  fy: 435.20
  cx: 367.45
  cy: 252.20
  focal_x_baseline: 47.91   # = fx * baseline
  depth_threshold: 40
```

### 2.5 RGB-D 示例

```cpp
// rgb_img 与 depthmap 必须像素对齐
// depthmap 为 CV_32F，真实深度 = pixel_value / depthmap_factor_
slam.depthmap_factor_ = 5000.0;  // TUM RGB-D 典型值
auto pose = slam.feed_RGBD_frame(rgb, depth, timestamp);
```

### 2.6 纯定位模式（Atlas，加载已有地图）

```cpp
slam.startup(false);  // need_initialize = false
slam.load_map_database("saved_map.msgpack");
slam.disable_mapping_module();  // 关闭建图，仅跟踪

// 丢失时可请求重定位
Mat44_t prior_pose = ...;
slam.relocalize_by_pose(prior_pose);
```

### 2.7 运行时开关

| API | 作用 |
|-----|------|
| `enable_mapping_module()` / `disable_mapping_module()` | 开/关局部建图 |
| `enable_loop_detector()` / `disable_loop_detector()` | 开/关回环检测 |
| `enable_temporal_mapping()` | 启用时序关键帧模式 |
| `pause_tracker()` / `resume_tracker()` | 暂停/恢复跟踪 |
| `request_reset()` | 重置地图与状态 |

### 2.8 验证定位是否正常

| 检查项 | 方法 |
|--------|------|
| 初始化成功 | `tracking_state_ == Tracking`（内部状态，可通过 frame_publisher 观察） |
| 位姿连续 | 相邻帧 $T_{cw}$ 变化平滑，无跳变 |
| 跟踪路标数 | `num_tracked_lms` ≥ 阈值（通常 > 50） |
| 回环生效 | `loop_detector_is_enabled()` 且地图无明显重影 |
| 地图可持久化 | `save_map_database` / `load_map_database` 往返一致 |

### 2.9 AMCL 配置预览（待集成）

```lua
-- config/localization/localization.lua
AUTONOMY_LOCALIZATION = {
    default_algorithm = "amcl",
    enabled = true,
    amcl = AMCL_OPTIONS,  -- 见 config/localization/amcl/amcl.lua
}
```

AMCL 集成后典型启动顺序：`MapServer` 发布 `/map` → 里程计 + `/scan` → AMCL 发布 `map→odom` TF。详见 [§7 AMCL](07_amcl.md)。

完整说明见 [Atlas 使用指南](atlas/guide.md)。
