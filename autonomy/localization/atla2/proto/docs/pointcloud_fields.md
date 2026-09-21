# PointCloud2 点时间戳字段约定

Atla2 LIO / LO / LIVO deskew 依赖**逐点时间**。点云一律使用 automsgs `sensor_msgs.PointCloud2`。

## 字段名（按优先级查找）

| 优先级 | `PointField.name` | 类型建议 | 含义 |
|--------|-------------------|----------|------|
| 1 | `timestamp` | `FLOAT64` | 绝对时间（秒，Unix 或与 scan 同步时钟） |
| 2 | `time` | `FLOAT32` / `FLOAT64` | 相对 scan 起始的偏移（秒） |
| 3 | `t` | `FLOAT32` | 同 `time`（部分 Livox 驱动） |

若三者皆无：整帧使用 `LidarScan.header.stamp`（end-of-scan），`t_begin` 仍用于匀速插值假设。

## 与 `LidarScan` 关系

```text
LidarScan.header.stamp  = end-of-scan
LidarScan.t_begin       = begin-of-scan
LidarScan.cloud          = PointCloud2（可含逐点 timestamp）
```

Deskew：点时刻 `t_i ∈ [t_begin, stamp]`，用 IMU 轨迹插值到 `stamp` 或 body 系。

## Intensity / 其它

| 字段 | 用途 |
|------|------|
| `x`,`y`,`z` | 必需，通常 FLOAT32 |
| `intensity` | 可选 FLOAT32 |
| `ring` / `laser_id` | 可选，线号 |

C++ `PointXYZI.timestamp` 对应上表优先字段写入/读出。
