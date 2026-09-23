# Atlas visual frontend — Tracking Spec

按 ORB-SLAM3 的 Tracking / Frame / ORB 流重构。

## 1. Goal

`Tracker` 拥有 ORB-SLAM3 风格的 `Map` + `KeyFrame` / `MapPoint`；系统层通过 `ConsumePendingKeyframe` 把关键帧快照交给 `MapManager`。

## 2. Modes

| Factory id | Class | Sensor |
|------------|-------|--------|
| `vo` | `VisualOdometry` | `camera.sensor`: `mono` / `stereo` / `rgbd` |
| `vio` | `VisualInertial` | 同上，并加 IMU |

顶层也可直接用 `system/slam_system`（对应 ORB-SLAM3 `System`）。

## 3. 核心类型映射

| ORB-SLAM3 | Atlas |
|-----------|-------|
| `Tracking` | `frontend/tracking/tracker` |
| `Frame` | `frontend/tracking/frame` |
| `ORBextractor` | `frontend/feature/orb/orb_extractor` |
| `ORBmatcher` | `frontend/match/orb_matcher` |
| `KeyFrame` / `MapPoint` / `Map` | `map/` |
| `LocalMapping` | `map/local_mapping` |
| `System` | `system/slam_system` |

## 4. Track / Mapping / Backend（autolink 调度）

```
Track (caller) ──KF──▶ ThreadSafeQueue ──▶ LocalMapping::Run (ThreadPool)
                                              │
                                              └──KF──▶ ThreadSafeQueue ──▶ LoopClosing::Run
GrabImuData ──▶ ThreadSafeQueue ──▶ PreintegrateImu (Track 线程)
```

`SlamScheduler`（`autolink::base::ThreadPool`，默认 3 worker）承载 Mapping / Loop 长驻 `Run`；Tracking 不阻塞 BA。

`GrabImage*` → FBoW → `Track` → `InsertKeyFrame`（异步）：

1. 初始化（Stereo / Monocular+TwoView）
2. Relocalization / MotionModel / ReferenceKF（`SearchByBoW`）
3. `TrackLocalMap` → `OptimizeCurrentPose`
4. `CreateNewKeyFrame` → LocalMapping 队列
5. Mapping worker：Cull / Triangulate / Fuse / **Ceres LocalBA|InertialBA** → 投递 LoopClosing
6. Loop worker：FBoW + **Sim3** + Essential Graph

**优化**：`backend/optimizer` 仅 **Ceres**（无 g2o）。

### IMU 位姿优化

- 外参：`Tracker::SetImuCalib`（`Tcw ↔ Twb` via `Tcb`）；`Frame/KeyFrame::GetImuPose`
- IMU 初始化后 `OptimizeCurrentPose` → PoseInertial（LastFrame / LastKeyFrame）
- 残差：预积分 **Info** 加权 + BiasWalk/Prior；视觉在 Twb 上（`Tcw = Tcb * Twb^{-1}`）
- Mapping：`InitializeImu` → `FullInertialBA`；单目 `ScaleRefinement`
- Loop：IMU 图用 `OptimizeEssentialGraph4DoF`；合并用 `MergeInertialBA`

## 5. Handoff

`ConsumePendingKeyframe` → `MapManager`；回环由 LocalMapping 直接 `InsertKeyFrame` 到 LoopClosing。

## 6. 可视化 channels（`system/constants.hpp`）

| Channel | 消息 | 用途 |
|---------|------|------|
| `/tf` | TFMessage | map→body / map→camera |
| `.../trajectory` | Path | 轨迹 |
| `.../odometry` | Odometry | 位姿 |
| `.../local_map_points` | PointCloud2 | 局部特征点 |
| `.../global_map_points` | PointCloud2 | 全局地图点 |
| `.../loop_closure` | MarkerArray | 回环边 |
| `.../camera_frustums` | MarkerArray | 关键帧相机 |
| `.../current_camera` | Marker | 当前相机 |

调用：`SlamSystem::StartVisualization(node)`；`Track*` 内自动 `Publish`。

见 [orb_slam3_mapping.md](orb_slam3_mapping.md)。
