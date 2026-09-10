# IMU 与 GPS

## IMU

- **YAML 键**：`imu`
- **消息**：`sensor_msgs/Imu`
- **Backend**：`serial`（默认）、`can`、`realsense` / `orbbec`（板载）

### 串口（WitMotion 等）

```yaml
imu:
  - name: torso_imu
    enable: true
    channel:
      - /imu/torso
      - /imu/torso/raw
    port: /dev/ttyUSB0
    baudrate: 460800
    fps: 200
```

源码：`autodriver/imu/`（parser + serial/CAN driver）。传输经 `common::Stream`。

### CAN

```yaml
imu:
  - name: can_imu
    enable: true
    channel: /imu/can
    backend: can
    params:
      interface: can0
      accel_can_id: "0x100"
      gyro_can_id: "0x101"
```

### 板载（相机）

折叠写在 `camera.imu:` 下（见 [RealSense](camera/realsense.md) /
[Orbbec](camera/orbbec.md)），展开后为 `ImuModule`。

---

## GPS / GNSS

- **YAML 键**：`gps`
- **消息**：`sensor_msgs/NavSatFix`
- **Backend**：`serial`（NMEA）、`can`

```yaml
gps:
  - name: gps_main
    enable: true
    channel: /gps/fix
    port: /dev/ttyUSB2
    baudrate: 9600
```

源码：`autodriver/gps/`（`nmea_0183` + `gps/parser` 工厂）。  
扩展二进制协议：向 `GnssParserRegistry` 注册，不必改 Module。

## 相关

- [配置 · imu / gps](../guide/configuration.md)
- [后端 · serial / canbus](../guide/backends.md)
