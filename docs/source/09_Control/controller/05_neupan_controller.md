(neupan-controller)=
# 5. NeuPAN Controller

源码：`autonomy/control/controller/neupan_controller/`。基于 [NeuPAN](https://github.com/hanruihua/NeuPAN) 的障碍点距离网络 + 运动规划，输出差速 `(v, ω)`。

### 结构

```
SetPlan → PathCache
SetScan → LaserScan 快照
Tick → ObstacleExtractor → DistanceEncoder (ONNX) → MotionPlanner → Twist
```

| 模块 | 作用 |
|------|------|
| `path_cache` | 路径 SoA + lookahead |
| `obstacle_extractor` | LaserScan → Eigen 点 |
| `robot_footprint` | Polygon → 半空间 $Ap\le b$ |
| `distance_encoder` | ONNX 距离特征 |
| `motion_planner` | 距离门控跟踪 |

### 模型导出

```bash
python3 tools/neupan/export_distance_encoder_onnx.py \
--checkpoint /path/to/model.pth \
--edge-count 4 \
--output /path/to/distance_encoder.onnx
```

`edge-count` 须与训练时足迹边数一致（矩形通常为 4）。

### 配置

示例：`autonomy/control/conf/neupan_controller_example.pb.txt`

- `controller_plugins: "neupan_controller:NeupanController"`
- `neupan_controller_options.neural_encoder_model_path`
