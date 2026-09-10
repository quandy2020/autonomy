# IMU / GPS

## IMU

| | |
|---|---|
| YAML | `imu` |
| 消息 | `sensor_msgs/Imu` |
| backend | `serial`（默认）、`can`、`realsense`（需 librealsense） |
| Registry | `ImuBackendRegistry` / `REGISTER_IMU_BACKEND` |
| 源码 | `autodriver/imu/`；serial → `SerialByteDriverBase`，can → `CanSensorDriverBase` |

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

板载：写在折叠 `camera.imu:`（见相机页）。`fps` → `publish_rate_hz`（串口仍跟硬件速率）。

## GPS

| | |
|---|---|
| YAML | `gps` |
| 消息 | `NavSatFix` |
| backend | `serial`（NMEA）、`can` |
| Registry | `GpsBackendRegistry` / `REGISTER_GPS_BACKEND` |
| 源码 | `autodriver/gps/` + `gps/parser`；serial/CAN 同上 CRTP 基类 |

```yaml
gps:
  - name: gps_main
    enable: true
    channel: /gps/fix
    port: /dev/ttyUSB2
    baudrate: 9600
```

工厂：`GnssParserRegistry::CreateParser("nmea"|"nmea0183")`。扩展二进制协议只加 Parser，不改 Module。
