# ORB-SLAM3 → Atlas 模块映射

| ORB-SLAM3 | Atlas | 状态 |
|-----------|-------|------|
| `System` | `system/slam_system` | ✓ |
| `Tracking` | `tracker` | ✓ RECENTLY_LOST / CreateMapInAtlas / ResetActiveMap |
| `LocalMapping::InterruptBA` | `InterruptBA` | ✓ 新 KF 时跳过当轮 BA |
| `Frame` stereo / undistort / bounds / frustum | ✓ | |
| `ComputeStereoFishEyeMatches` + lapping + `Nleft` | ✓ | |
| 右目 `IsInFrustum` + `SearchByProjection` | ✓ | |
| `SearchByProjection(LastFrame)` 双目鱼眼 + `Project` | ✓ | |
| `Fuse(bRight)` + KF `grid_right` / `GetRightPose` | ✓ | |
| `SearchForTriangulation` ll/lr/rl/rr + `EpipolarConstrain` | ✓ | |
| `SearchByBoW` 双目左右分轨（KF↔Frame / KF↔KF 仅左） | ✓ | |
| Reloc `SearchByProjection(Frame,KF)` + 粗/细投影补点 | ✓ | |
| Loop `Fuse(Scw)` + `SearchAndFuse` Replace | ✓ | |
| `SearchBySim3` 引导匹配（EstimateSim3） | ✓ | |
| CorrectLoop Sim3 传播 + MapPoint 校正后 Fuse | ✓ | |
| CorrectLoop 已匹配 MP Replace/Add + DistinctiveDesc | ✓ | |
| `MapPoint::ComputeDistinctiveDescriptors` 最小中位 Hamming | ✓ | |
| `MapPoint::UpdateNormalAndDepth` 全观测法向平均 | ✓ | |
| `CreateNewKeyFrame` 近点 depth 排序 + 填满 100 | ✓ | |
| `UpdateLastFrame` VO 时域点（仅左 / thDepth） | ✓ | |
| `LocalMapping::EmptyQueue` + `RequestPause`/`Release` | ✓ | |
| Loop 校正前暂停 LocalMapping（EmptyQueue） | ✓ | |
| `only_tracking` VO 双假设（mbVO） | ✓ | |
| `LocalMapping::SetNotStop`（CreateNewKeyFrame） | ✓ | |
| Loop 后异步 GBA（`LaunchGlobalBA`） | ✓ | |
| `KeyFrame::GetChildren` 生成树查询 | ✓ | |
| `KeyFrameCulling` 双目 octave + `imu_init` 次序 | ✓ | |
| PoseOpt / LocalBA / GlobalBA 右目 ToBody | ✓ | |
| Kannala 可微投影（Ceres `ProjectPixel`） | ✓ 对齐 `GeometricCamera::Project` | |
| `MapPoint::GetReplaced` + CheckReplaced | ✓ | |
| MLPnP / GeometricTools / ORB / FBoW / matcher | ✓ | |
| g2o → **Ceres** | 惯性局部/全局 BA：左目、双目 \((u,v,u_r)\)、右目与 IMU 预积分同一问题 | ✓ |
| Loop / KF DB / Camera Factory / IMU / Settings | ✓ | |
| `"vio"` 工厂 | `camera.sensor`: `mono` / `stereo` / `rgbd` → IMU 单目 / 双目 / RGB-D | ✓ |
| Atlas IO | JSON **v5**（BoW + Camera + IMU 预积分 / prev-next） | ✓ |
| Viewer | — | 不做 |

**评测**：synthetic ATE **0.0114 m**。EuRoC 绝对轨迹误差尚未在本机序列上测过（需要 `atlas_dataset` 与数据集，入口 `test/app/run_ate.sh`）。

**算法**：位姿惯性在重初始化时跳过宽松内点恢复，并在上一帧上加 `EdgePriorPoseImu`；纯视觉 BA 一次求解后按 \(\chi^2\) 剔点；本质图相对边用校正前位姿；全局 BA 先备份再沿生成树传播；惯性合并走 `MergeLocal2`（缩放当前地图、重接生成树、焊接 BA）；初始化运动不足时置 `bad_imu`，由跟踪线程重置活动地图。

**系统接口**：TUM / EuRoC / KITTI 轨迹、`MapChanged`、`GetTrackingState`、`isLost`、`isFinished`、`GetTimeFromIMUInit`、`Reset`、`ResetActiveMap`、`ChangeCalibration` 已挂到 `SlamSystem`。地图读写仍是 JSON。

**与 g2o 的差别**：求解器固定为 Ceres，消元顺序不同。上一帧先验的信息矩阵由当前帧雅可比堆成。EuRoC 绝对轨迹误差尚未实测。
