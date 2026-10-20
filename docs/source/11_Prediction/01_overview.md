(prediction-overview)=
# 1. 模块概览

目标形态是：感知给障碍，预测出轨迹 / 意图，规划再用。眼下 `PredictionServer` 只是壳，`PredictionOptions` 也几乎是空的。

真正能复用的在别处：`autonomy/common/motion_model/`（CV / CA、差速、静止等）和 `common/state_estimation/kalman_filter/`。导航主路径还是 costmap → Planning → Control，没有接动态预测。

```
autonomy/prediction/
├── prediction_server.*
├── common/prediction_interface.*
└── proto/prediction_options.proto
```

运动模型用法见 [06](06_motion_models.md)。
