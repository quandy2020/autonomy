# 9. Sensor、Vehicle、Audio

三个支撑目录，都在 `autonomy/` 下。

**Sensor**（`sensor/`）把激光、IMU、里程计、点云等收成可注入 localization / control / costmap 的数据结构，时间对齐走 collator / consumer。

**Vehicle**（`vehicle/`）里 `VehicleServer` 拿上层速度指令，经 `KinematicsControl` 限幅，再交给具体 `VehicleInterface`（差速、阿克曼等）。选项在 `proto/vehicle_options.proto`；真硬件驱动在顶层 `autodriver/`。

**Audio**（`audio/`）吃麦阵 `/mic/cabin/audio`，出方向 / 运动检测，可选 sherpa-onnx ASR：

```text
/mic/cabin/audio → AudioComponent
                 → /audio/detection
                 → /audio/asr（可选）
```

开 ASR：`cmake -DBUILD_SHERPA_ONNX=ON`，模型路径写在 `conf/audio.pb.txt`。细节见 `audio/README.md`。
