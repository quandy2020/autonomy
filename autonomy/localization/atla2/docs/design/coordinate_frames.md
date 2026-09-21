# 坐标系与外参

## 1. 约定

| 坐标系 | 符号 | 说明 |
|--------|------|------|
| World / Odom | `W` | 里程计世界系，右手，重力大致 −Z 或按平台约定 |
| Body | `B` | 机体 / 飞控参考点 |
| IMU | `I` | IMU 测量系 |
| Camera | `C` | 光心，OpenCV：x 右、y 下、z 前 |
| Lidar | `L` | 雷达光学/几何中心 |
| GPS antenna | `G` | 天线相位中心 |
| ENU | `E` | 本地切平面，GPS 融合用 |

位姿表示：**`T_parent_child`** = 子系在父系下的位姿（与 `TransformTree` 一致）。  
代码类型：`SE3 = Eigen::Isometry3d`。

## 2. 外参树

`TransformTree` 存边 `parent → child`：

```text
body
├── imu
├── cam
├── lidar
└── gps
```

查询 `Lookup(a, b, &T_a_b)` 沿树组合。缺边返回 `false`。

配置示例（`config/sensors/*/…yaml`）：

```yaml
extrinsic:
  translation: [0.05, 0.0, 0.02]   # meters
  rotation_xyzw: [0.0, 0.0, 0.0, 1.0]
```

## 3. 时间

- `TimeStamp`：纳秒整数（Unix 或同步时钟）
- 软同步：`WithinTol(a, b, tol_ns)`，默认 `sync_tol_ms`（平台 YAML）
- 雷达 deskew：点时间戳相对 scan 起始，用 IMU 轨迹插值

## 4. 输出位姿

`OdometryResult.pose` = **`T_W_B`**（世界 ← 机体），速度在世界系（或 body，以后端约定为准；当前骨架为世界系平移速度）。

## 5. 标定产物落盘

| 文件 | 内容 |
|------|------|
| `sensors/camera/*.yaml` | 内参 + `T_B_C` |
| `sensors/imu/*.yaml` | 噪声 + `T_B_I` |
| `sensors/lidar/*.yaml` | `T_B_L` + 量程 |
| 标定工具 `--out` | 覆盖上述 extrinsic |

详见 [../calibration/guide.md](../calibration/guide.md)。
