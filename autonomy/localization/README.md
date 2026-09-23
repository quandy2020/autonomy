# localization

进程级定位：`autonomy.localization` → `LocalizationServer`。

| 后端 / 库 | 用途 |
|-----------|------|
| **Cartographer** | 2D/3D lidar SLAM |
| **Lightning** | 独立 LIO（`lightning/`） |
| **atlas** | VO / VIO 多传感器库（`atlas/`，`cli/` / `node/` 入口） |

```text
autonomy/localization/
├── localization_main.cpp
├── localization_server.*      # Cartographer | Lightning 门面
├── atlas/                     # VO/VIO 库（演进中）
├── cartographer/
├── lightning/
└── conf/
    ├── cartographer/
    └── lightning/
```

## 快速启动

**Cartographer**

```bash
autonomy.localization \
  --localization_mode=cartographer \
  --configuration_directory=autonomy/localization/conf/cartographer \
  --configuration_basename=autosim_2d.lua
```

或：`autolink launch start localization.launch` / `localization_cartographer.launch`

**Lightning LIO**

```bash
autonomy.localization \
  --localization_mode=lightning \
  --lightning_config=autonomy/localization/conf/lightning/autosim.yaml \
  --lightning_imu_topic=/imu \
  --lightning_lidar_topic=/points
```

或：`localization_lightning.launch`

## 常用 flags

| Flag | 说明 |
|------|------|
| `--localization_mode` | `cartographer` \| `lightning` |
| Cartographer | `--configuration_directory` / `--configuration_basename` / `--save_state_filename` |
| Lightning | `--lightning_config` / `--lightning_imu_topic` / `--lightning_lidar_topic` |

atlas 结构见 [atlas/README.md](atlas/README.md)。
