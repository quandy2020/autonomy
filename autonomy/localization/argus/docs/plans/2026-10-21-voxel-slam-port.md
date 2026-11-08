# Plan: Voxel-SLAM → Argus（2026-10-21）

状态：算法核心已与上游对齐（见 `docs/13_voxel_slam.md`），不含 ROS。

可选后续：

1. `offline_voxel_slam` 接 KITTI / Spires dataset 回放
2. 评估与 LIVO 统一体素图抽象
3. OctreeGba 节点池回收（降低 HBA 分配开销）
