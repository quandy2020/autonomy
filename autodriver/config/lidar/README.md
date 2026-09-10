# Slamtec (RPLidar) vendor **device params**

Main process config: `config/autodriver_hardware.yaml` (`lidar_2d` entries).
Point each device at a model file with `params_file:`:

```yaml
lidar_2d:
  - name: front
    enable: true
    channel: /lidar/front/scan   # or /scan
    backend: rplidar             # alias: slamtec
    port: /dev/ttyUSB0
    baudrate: 115200             # optional; overwritten by params_file baud
    params_file: lidar/slamtec/a1.yaml
    params:
      frame_id: laser
```

| Model | File | baud | notes |
|---|---|---|---|
| A1 | `a1.yaml` | 115200 | |
| A2 / A2M8 | `a2.yaml` | 115200 | A2M7/A2M12 → 256000 |
| A3 | `a3.yaml` | 256000 | `scan_mode: Sensitivity` |

Usage aligned with rplidar_ros; publishes `sensor_msgs/LaserScan` on Autolink.
Build: install SDK (`scripts/install_rplidar_sdk.sh`), then
`-DAUTODRIVER_WITH_RPLIDAR=ON`. Optional udev: `scripts/create_udev_rules.sh`
→ `/dev/rplidar`.
