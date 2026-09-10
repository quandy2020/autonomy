# RPLidar (Slamtec) 2D lidar

`backend: rplidar` (alias `slamtec`) for `lidar_2d`.

Requires an **installed** Slamtec `rplidar_sdk` at build time
(`AUTODRIVER_WITH_RPLIDAR=ON`).

```bash
# Download from https://github.com/slamtec/rplidar_sdk , build, install to /usr/local
./scripts/install_rplidar_sdk.sh
# Or: PREFIX=/usr/local RPLIDAR_SDK_REF=master ./scripts/install_rplidar_sdk.sh

# Optional: fixed device node /dev/rplidar
./scripts/create_udev_rules.sh
```

Override search with `-DRplidarSDK_ROOT=/usr/local` or `RPLIDAR_SDK_DIR`.

| File | Role |
|------|------|
| `serial_driver.*` | Serial channel + grab loop |
| `convert.*` | HQ nodes → `sensor_msgs/LaserScan` (rplidar_ros `publish_scan`) |

Params (rplidar_ros aligned): `device`/`port`, `baud`, `model`, `frame_id`,
`scan_mode`, `inverted`, `angle_compensate`, `max_distance`, `motor_pwm`, etc.
See `config/lidar/slamtec/{a1,a2,a3}.yaml`.
