# Intel RealSense

深度相机系列（本仓库以 **D455** 为主）。经 librealsense2 开流，多路
`camera` / `point_cloud` / 板载 `imu` 共用一个 `device_hub`。

## 简介

- **消息**：`sensor_msgs/Image`、camera_info、深度点云 `PointCloud2`、`Imu`
- **Backend**：`realsense`
- **源码**：`autodriver/camera/realsense/`
- **厂商参数**：`config/camera/realsense/d455.yaml`

## 依赖

| 项 | 说明 |
|---|---|
| CMake | `AUTODRIVER_WITH_REALSENSE=ON`（默认） |
| 库 | librealsense2（`find_package(realsense2)`） |
| 权限 | 用户加入 `plugdev` / 厂商 udev；USB3 |

未找到 SDK 时驱动 stub，`Create`→`nullptr`。

## 配置（推荐折叠写法）

一台物理机一条 `camera` 条目，loader 展开为多路 Sensor：

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
      - stream: color
        channel: /camera/color/image_raw
        frame_id: camera_color_optical_frame
      - stream: depth
        channel: /camera/depth/image_rect_raw
        frame_id: camera_depth_optical_frame
      - stream: ir1
        channel: /camera/infra1/image_rect_raw
        frame_id: camera_infra1_optical_frame
      - stream: ir2
        channel: /camera/infra2/image_rect_raw
        frame_id: camera_infra2_optical_frame
      - stream: aligned_depth_to_color
        channel: /camera/aligned_depth_to_color/image_raw
        frame_id: camera_color_optical_frame
    point_clouds:
      - name: points
        channel: /camera/depth/color/points
        frame_id: camera_depth_optical_frame
    imu:
      channel: /camera/imu
      frame_id: camera_imu_optical_frame
```

通道命名对齐 **realsense-ros**。`params_file` 含激光功率、滤波、`model: D455` 等；
`serial` / `index` 可选。

## 使用要点

1. 默认主配置已启用 D455；换机改 `params_file` 或分辨率即可。
2. 同机多流不要开多个独立 pipeline——折叠配置共享 hub。
3. 验证脚本：`scripts/verify_realsense_d455.sh`（若存在）。

## 相关

- [配置 · camera](../guide/configuration.md#camera)
- [后端 · RealSense](../guide/backends.md)
