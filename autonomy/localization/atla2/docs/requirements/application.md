# 应用需求

面向具体机型与任务，将业务场景映射到 Atla2 平台配置与模式。

## 1. 应用场景矩阵

| 场景 | 典型平台 | 模式 | 平台 YAML | 说明 |
|------|----------|------|-----------|------|
| 室内视觉定位 | 小型多旋翼 | VIO | `drone_vision_only.yaml` | 无雷达，Ceres 窗口 |
| 纯视觉调试 | 多旋翼 | VO | `drone_vo.yaml` | 无 IMU 前端 |
| 室外激光导航 | 多旋翼 / 车载吊舱 | LIO | `drone_lidar_imu.yaml` | Mid-360 + IEKF |
| 纹理弱 + 结构弱 | 巡检无人机 | LIVO loose | `drone_livo.yaml` | 视觉激光互补 |
| VTOL 长航时 | 垂起固定翼 | LIVO + GPS | `vtol_livo_gps.yaml` | GPS/气压辅助 |

## 2. 传感器组合需求

| 组合 | 必需 | 可选 | 融合 |
|------|------|------|------|
| VO | Camera | — | — |
| VIO | Camera + IMU | — | 紧（图） |
| LIO | Lidar + IMU | — | IEKF |
| LIVO | Camera + Lidar + IMU | GPS / Baro / Flow | loose→tight |
| VTOL | LIVO + GPS | Baro | EKF 位置/高度 |

## 3. 运行时接口需求（对飞控）

| 输出 | 频率目标 | 用途 |
|------|----------|------|
| `pose` + `velocity` | ≥ 控制周期 | 位置/速度环 |
| `cov` | 同 pose | 健康与融合权重 |
| `SlamState` | 事件 | 任务状态机 |
| 局部地图 / 高程（可选） | 较低 | 避障 / 规划 |

输入：IMU 高率；相机 20–60 Hz；雷达 10 Hz；GPS 1–10 Hz。

## 4. 环境与工况

| 工况 | 需求 |
|------|------|
| 强光 / 暗光 | VIO 特征失败 → 降级 LIO |
| 烟尘 / 雾 | 雷达噪声增大 → 门控拒识 |
| 高速运动 | deskew 必须开启 |
| 温升 | 嵌入式跑 `run_embedded.sh`，热节流降频 |

## 5. 配置落地检查清单

部署新机型时：

1. [ ] 复制/新建 `config/platforms/<name>.yaml`  
2. [ ] 填写 `sensors/*` 外参与噪声  
3. [ ] 选定 `frontend.mode` 与 `backend.type`  
4. [ ] `scripts/calibrate.sh` 完成关键外参  
5. [ ] `atla2_offline` 合成冒烟  
6. [ ] `tools/benchmark/runners/run_offline.sh` 对目标数据集打基线  

## 6. 非目标（当前版本）

- 多机协同建图  
- 语义 SLAM  
- 云端大地图服务  

以上列入后续路线，不阻塞 P0 交付。
