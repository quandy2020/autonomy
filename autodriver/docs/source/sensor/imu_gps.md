# IMU / GPS

## IMU

| | |
|---|---|
| YAML | `imu` |
| 消息 | `sensor_msgs/Imu` |
| backend | `serial`（默认）、`can`、`realsense`（依赖 librealsense） |
| Registry | `ImuBackendRegistry` / `REGISTER_IMU_BACKEND` |
| 源码 | `autodriver/imu/`；serial 经 `SerialByteDriverBase`，can 经 `CanSensorDriverBase` |

```yaml
imu:
  - name: torso_imu
    enable: true
    channel: [/imu/torso, /imu/torso/raw]
    port: /dev/ttyUSB0
    baudrate: 460800
    fps: 200
  - name: can_imu
    enable: false
    backend: can
    channel: /imu/can
    params: {interface: can0, accel_can_id: "0x100", gyro_can_id: "0x101"}
```

板载 IMU 写在折叠配置的 `camera.imu:`（见相机页）。`fps` 映射为 `publish_rate_hz`；串口路径的实际发布仍跟随硬件采样率。

## GPS

| | |
|---|---|
| YAML | `gps` |
| 消息 | `NavSatFix` |
| backend | `serial`（NMEA）、`can` |
| Registry | `GpsBackendRegistry` / `REGISTER_GPS_BACKEND` |
| 源码 | `autodriver/gps/` 与 `gps/parser`；serial / CAN 使用与 IMU 相同的 CRTP 基类 |

```yaml
gps:
  - name: gps_main
    enable: true
    channel: /gps/fix
    port: /dev/ttyUSB2
    baudrate: 9600
```

工厂：`GnssParserRegistry::CreateParser("nmea"|"nmea0183")`（**语句解析**，与传输后端正交）。扩展二进制协议时仅新增 Parser，无需修改 Module。
