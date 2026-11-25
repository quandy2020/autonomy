# localization 使用文档

`autonomy.localization` 进程通过 `--localization_mode` 选择后端。同一时刻只起一个定位后端。

按编号阅读：

1. [总览与选型](01_overview.md)
2. [编译与公共环境](02_setup.md)
3. [数据集下载与发布](03_datasets.md)
4. [Atlas 视觉 / VIO](04_atlas.md)
5. [Argus 激光–惯性 Voxel-SLAM](05_argus.md)
6. [Cartographer 2D](06_cartographer.md)
7. [排查](07_troubleshooting.md)

实现细节：

- [Atlas](../atlas/README.md)
- [Argus](../argus/README.md)
- [Voxel-SLAM 管线](../argus/docs/13_voxel_slam.md)
- 仓库文档：[06 Localization](../../../docs/source/06_Localization/00_guide.md)、[数据集工具](../../../docs/source/18_Tools/08_datasets.md)
