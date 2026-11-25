# localization

`autonomy.localization` 提供三种后端：

- **Atlas**：单目、双目、RGB-D 视觉 SLAM，以及对应的视觉惯性模式。
- **Argus**：激光–惯性 Voxel-SLAM（当前 launch 为纯 LIO）。
- **Cartographer**：AutoSim 2D 激光定位与建图。

## 使用

操作步骤（编译、下载数据、Atlas / Argus / Cartographer、Autoviz、排查）写在：

**[docs/](docs/README.md)**

同一时刻只起一个定位后端。launch 必须在 `/workspace/autonomy/src/autonomy` 启动，否则相对配置路径无法解析。

```bash
docker exec -it SpaceHero /bin/bash
cd /workspace/autonomy
colcon build --packages-up-to autonomy
```

数据集一键下载：`scripts/download_datasets.sh`（参数见 [docs/03_datasets.md](docs/03_datasets.md)）。

## 实现与仓库文档

- [Atlas](atlas/README.md)
- [Argus](argus/README.md)
- [Argus Voxel-SLAM](argus/docs/13_voxel_slam.md)
- [06 Localization](../../docs/source/06_Localization/00_guide.md)
- [数据集发布](../../docs/source/18_Tools/08_datasets.md)
- [AutoSim Atlas 验证](../../autosim/docs/13_atlas_validation.md)
