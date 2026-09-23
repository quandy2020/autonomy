# Atlas

多传感器 SLAM 库（先 VO / VIO）。**核心视觉前端按 ORB-SLAM3 重构**。无 ROS 依赖；进程入口在 `cli/` / `node/`。

| | |
|---|---|
| 模式 id | `"vo"` \| `"vio"`（工厂注册） |
| 命名空间 | `autonomy::localization::atlas` |
| 构建 | 父级 `autonomy/localization` glob 源码 |

## 目录

```text
atlas/
├── common/                 # 类型、配置
├── sensor/                 # SensorData
├── frontend/
│   ├── feature/orb/        # ORBextractor + FBoW OrbVocabulary ✓
│   ├── match/              # ORBmatcher（投影 / Fuse / SearchByBoW）✓
│   └── tracking/           # Frame / Tracker / TwoView / VO / VIO ✓
├── backend/                # Ceres Optimizer + LoopClosing ✓
├── map/                    # Map / KeyFrame / MapPoint / LocalMapping / KF-DB ✓
├── cli/  node/
├── config/
├── test/unit/
└── docs/design/
```

## 快速用法

```cpp
#include "autonomy/localization/atlas/system/slam_system.hpp"

autonomy::localization::atlas::SlamSystem system;
autonomy::localization::atlas::AtlasConfig config;
system.Init(config, autonomy::localization::atlas::SlamSystem::Sensor::kRgbd);
system.TrackRgbd(rgb, depth, timestamp);
```

或工厂：

```cpp
#include "autonomy/localization/atlas/frontend/factory.hpp"

auto frontend = autonomy::localization::atlas::CreateFrontend(config);
frontend->Process(sensor_data);
```

设计说明：[docs/design/visual_frontend.md](docs/design/visual_frontend.md) · 模块映射：[docs/design/orb_slam3_mapping.md](docs/design/orb_slam3_mapping.md)。
