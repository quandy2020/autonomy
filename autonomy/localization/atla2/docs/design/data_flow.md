# 数据流

## 1. 端到端一步

```text
[驱动/回放] → SensorSuite / Push*
                ↓
           SensorSync::TryPop → SensorData
                ↓
           Frontend::Process(SensorData)
                ↓
           OdometryResult (pose, v, bias, cov, landmarks|cloud)
                ↓
           Degradation::Evaluate → HealthReport
                ↓
           Backend::UpdateWithSensors(odom, data)
                ↓
           MapManager::UpdateFromOdometry
                ↓
           GetOdometry → 控制 / 可视化 / 日志
```

## 2. SensorData 载荷

| 字段 | 条件 | 消费者 |
|------|------|--------|
| `imu[]` | VIO/LIO/LIVO | 预积分、deskew、IEKF |
| `image` | VO/VIO/LIVO | 特征跟踪 |
| `lidar` | LIO/LIVO | 配准 / 地图 |
| `t` | 始终 | 同步参考时刻 |

## 3. 前端输出 → 后端输入

后端 **不解析图像/点云语义**（除 graph 的 VIO 路径可再吃 IMU）：统一吃 `OdometryResult`。

```text
VO/VIO ──landmarks──► map/landmark + visual factors
LIO   ──local_map──► map/point_cloud + IEKF
LIVO  ──两者──────► 融合后再写入 map
```

## 4. 地图写入

| 触发 | 行为（当前骨架） |
|------|------------------|
| 每帧 odom | `UpdateFromOdometry` 更新局部云 / 路标缓存 |
| 关键帧策略 | `AddKeyframe`（平移/转角阈值，见 `config/map`） |

## 5. 离线 vs 在线

| 路径 | 数据来源 | 入口 |
|------|----------|------|
| 离线合成 | `offline_runner` 内构造 `SensorData` | `SlamSystem::Step(data)` |
| 离线数据集 | 转换后 Push*（规划） | 同左 |
| 在线 | 驱动 → suite → sync | `air_slam_node` + Push* |

## 6. 评测数据流

```text
runner → est.txt / run.log
           ├─► metrics/accuracy.py   (vs gt.txt)
           ├─► metrics/efficiency.py
           ├─► metrics/robustness.py
           └─► metrics/thermal.py
                 ↓
           tools/benchmark/reports/<dataset>/<ts>/
```
