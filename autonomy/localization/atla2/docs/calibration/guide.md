# 标定指南

## 1. 为何标定

外参误差会直接表现为：

- VIO 尺度/重力对齐差  
- LIO deskew 残影、z 漂移  
- LIVO 视觉–激光关联失败  

所有外参统一为 **`T_body_sensor`**，写入 `config/sensors/*/…yaml`。

## 2. 工具入口

```text
apps/calibration_tool/
├── camera_imu_calib/
├── lidar_imu_calib/
├── camera_lidar_calib/
└── joint_calib/
```

脚本封装：

```bash
./scripts/calibrate.sh camera_imu <bag> [--out camera_imu_extrinsic.yaml]
./scripts/calibrate.sh lidar_imu <bag>
./scripts/calibrate.sh camera_lidar <bag>
./scripts/calibrate.sh joint <bag>
```

当前为 **CLI 骨架**（解析参数并占位）；求解器接入前可用 Kalibr / LI-Calib 等离线结果手工填 YAML。

## 3. 推荐流程

```text
1. 相机内参（棋盘 / AprilTag）→ sensors/camera/*.yaml
2. 相机–IMU 外参 + 时间偏移 → camera_imu
3. 激光–IMU 外参 → lidar_imu
4. 相机–激光验证 / 微调 → camera_lidar
5. （可选）三者联合 → joint
6. 装入 platforms/*.yaml 引用，跑 offline 冒烟
```

## 4. YAML 字段

```yaml
# 例：sensors/camera/front.yaml
intrinsics: { fx, fy, cx, cy }
distortion: { model, k1, k2, p1, p2 }
extrinsic:
  translation: [x, y, z]       # meters, body ← cam
  rotation_xyzw: [x, y, z, w]
```

IMU / Lidar 同理，见各默认 yaml。

## 5. 验证

| 检查 | 方法 |
|------|------|
| 重投影 | 标定板角点误差 < 1 px（经验） |
| 激光–相机 | 投影边缘对齐 |
| 时间偏移 | 快摇运动，看残差尖峰是否降低 |
| 端到端 | 短轨迹 ATE 对比标定前后 |

## 6. 注意事项

- 先刚性固定传感器再标定；飞行中形变需定期复标  
- 左手系 / 光学系约定见 [../design/coordinate_frames.md](../design/coordinate_frames.md)  
- 标定 bag 与运行固件版本记入数据集元数据  

## 7. 相关

- [../design/coordinate_frames.md](../design/coordinate_frames.md)  
- [../../apps/calibration_tool/README.md](../../apps/calibration_tool/README.md)  
