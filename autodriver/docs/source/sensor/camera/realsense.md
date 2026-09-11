# Intel RealSense

D455 为主。基于 librealsense2；同机多路共享 `device_hub`。

| | |
|---|---|
| 消息 | Image、camera_info、PointCloud2、Imu |
| backend | `realsense` |
| 源码 | `autodriver/camera/realsense/` |
| params | `config/camera/realsense/d455.yaml` |
| CMake | `AUTODRIVER_WITH_REALSENSE` + `find_package(realsense2)` |

## 折叠配置（推荐）

展开 id：`camera/<name>_<stream>`、`camera/<name>_points`、`imu/<name>_imu`。

```yaml
camera:
  - name: realsense_d455
    enable: true
    backend: realsense
    params_file: camera/realsense/d455.yaml
    width: 848
    height: 480
    fps: 30
    streams:
      - {stream: color, channel: /camera/color/image_raw, frame_id: camera_color_optical_frame}
      - {stream: depth, channel: /camera/depth/image_rect_raw, frame_id: camera_depth_optical_frame}
      - {stream: ir1, channel: /camera/infra1/image_rect_raw, frame_id: camera_infra1_optical_frame}
      - {stream: ir2, channel: /camera/infra2/image_rect_raw, frame_id: camera_infra2_optical_frame}
      - {stream: aligned_depth_to_color, channel: /camera/aligned_depth_to_color/image_raw, frame_id: camera_color_optical_frame}
    point_clouds:
      - {name: points, channel: /camera/depth/color/points, frame_id: camera_depth_optical_frame}
    imu:
      channel: /camera/imu
      frame_id: camera_imu_optical_frame
```

通道对齐 realsense-ros。子项可 `enable: false`。无折叠键时仍支持扁平单流。

## params_file 要点（d455.yaml）

`model`、`emitter_enabled`、`laser_power`、`visual_preset`、深度滤波（spatial/temporal/hole_fill）、可选 `serial`/`index`。

## 注意

须使用 USB3，并确认用户组权限。请勿与 Orbbec 同时占用相同的 `/camera/*` 话题。冒烟验证：`scripts/verify_realsense_d455.sh`。
