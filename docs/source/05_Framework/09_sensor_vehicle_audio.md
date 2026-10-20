# 9. Sensor、Vehicle、Audio

导航栈旁边三个支撑目录，都在 `autonomy/` 下。车不动或 costmap 空，往往要查这里而不只是 planning。 导航不动或 costmap 空时，沿 sensor→localization/costmap 与 vehicle→底盘 两条链查，而不只盯 planning。Audio 与闭环解耦，不参与路径跟踪。

**Sensor**（`sensor/`）把激光、IMU、里程计、点云等收成可注入 localization / control / costmap 的数据结构，时间对齐走 collator / consumer。

**Vehicle**（`vehicle/`）里 `VehicleServer` 拿上层速度指令，经 `KinematicsControl` 限幅，再交给具体 `VehicleInterface`（差速、阿克曼等）。选项在 `proto/vehicle_options.proto`；真硬件驱动在顶层 `autodriver/`。

**Audio**（`audio/`）吃麦阵 `/mic/cabin/audio`，出方向 / 运动检测，可选 sherpa-onnx ASR：

```text
/mic/cabin/audio → AudioComponent
                 → /audio/detection
                 → /audio/asr（可选）
```

开 ASR：`cmake -DBUILD_SHERPA_ONNX=ON`，模型路径写在 `conf/audio.pb.txt`。细节见 `audio/README.md`。
