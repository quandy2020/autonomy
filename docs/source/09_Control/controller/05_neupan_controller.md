(neupan-controller)=
# 5. NeuPAN Controller

激光障碍点 + **ONNX 距离编码** + 运动规划，输出差速 $(v,\omega)$。基于 [NeuPAN](https://github.com/hanruihua/NeuPAN)。源码：`neupan_controller/`。

激光障碍点经 ONNX 距离编码再进运动规划，输出线速度与角速度。有稳定 LaserScan 且模型已导出时，可在传统几何控制外接学习距离场；scan 不对齐或无 ONNX 时不要选。

## 何时用

有稳定 LaserScan、已导出距离编码 ONNX；想在传统几何控制外接学习距离场。无模型或 scan 不对齐时不要选。

## 流水线

```text
SetPlan → PathCache
SetScan → LaserScan 快照
Tick → ObstacleExtractor → DistanceEncoder (ONNX) → MotionPlanner → Twist
```

| 模块 | 作用 |
|------|------|
| `path_cache` | 路径 SoA + lookahead |
| `obstacle_extractor` | Scan → 点集 |
| `robot_footprint` | 多边形 → 半空间 |
| `distance_encoder` | ONNX 推理 |
| `motion_planner` | 距离门控跟踪 |

## 模型导出

```bash
python3 tools/neupan/export_distance_encoder_onnx.py \
  --checkpoint /path/to/model.pth \
  --edge-count 4 \
  --output /path/to/distance_encoder.onnx
```

`edge-count` 须与训练足迹边数一致（矩形常为 4）。

## 配置

示例：`autonomy/control/conf/neupan_controller_example.pb.txt`

- `controller_plugins: "neupan_controller:NeupanController"`
- `neupan_controller_options.neural_encoder_model_path`

## 常见坑

| 现象 | 处理 |
|------|------|
| 推理失败 | ONNX 路径；输入维度 |
| 避障 odd | scan 坐标系与 footprint 不一致 |
| CPU 高 | 减小点云抽取密度 |

归属 [04](../04_controller_algorithms.md)。
