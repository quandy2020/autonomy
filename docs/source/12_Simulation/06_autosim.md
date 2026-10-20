# 6. AutoSim

`autosim`：autolink 仿真桥（fake / Habitat / Isaac）。向定位与导航发布 `/imu`、`/points`、相机等。

### 6.1 启动

```bash
# 仓库内（路径随布局调整）
CONFIG=src/autonomy/autosim/config/lightning.yaml \
src/autonomy/autosim/scripts/run.sh sim
```

Isaac：用 NVIDIA 容器；`python` 常被 alias 成 Kit，系统工具请用 `/usr/bin/python3`。

### 6.2 与定位联调

| 后端 | 配置提示 |
|------|----------|
| Atlas LIO | `atlas/config/autosim_lio.yaml`（相机关） |
| Argus | `localization_argus.launch`；话题对齐 `lio.yaml` |

`config/lightning.yaml` 为 AutoSim 场景配置名，**不是**定位后端。

典型话题：`/imu` · `/points` · `/camera/rgb/image_raw` · `/tf`。

### 6.3 目录

```text
autosim/
├── autosim/ # runtime · backends · bridge
├── config/ # lightning.yaml · isaac.yaml · …
└── scripts/run.sh
```

定位步骤：[Atlas](../06_Localization/08_atlas/guide.md) · [Argus](../06_Localization/06_argus.md)。
