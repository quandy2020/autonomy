# sensor

- `SyncedSensorPacket` — 软同步多传感器包（对齐 C++ `SensorData`）
- `LidarScan` — PointCloud2 + `t_begin` + model（deskew）
- `GpsSample` — NavSatFix + ENU/速度/HDOP
- `BarometerSample` — FluidPressure + 温度/高度
- `OpticalFlowSample` — 光流辅助
