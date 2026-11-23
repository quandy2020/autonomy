(neupan-controller)=
# 5. NeuPAN Controller

激光障碍点 + **ONNX DUNE** + **NRMP（DAQP）** + PAN，输出差速 $(v,\omega)$ 或全向 $(v_x,v_y)$。基于 [NeuPAN](https://github.com/hanruihua/NeuPAN)。源码：`neupan_controller/`。QP 使用仓库内 `common/math/miqp`（DAQP），不依赖 OSQP / cvxpy。

激光障碍点经 ONNX 距离编码再进运动规划，输出线速度与角速度。有稳定 LaserScan 且模型已导出时，可在传统几何控制外接学习距离场；scan 不对齐或无 ONNX 时不要选。

## 何时用

有稳定 LaserScan、已导出距离编码 ONNX；想在传统几何控制外接学习距离场。无模型或 scan 不对齐时不要选。

## 流水线

```text
SetPlan → NeuPanPlanner::SetInitialPath
SetScan → LaserScan 快照
Tick → ObstacleExtractor → world 点 → NeuPanPlanner (PAN: DistanceEncoder + NRMP) → Twist
```

| 模块 | 作用 |
|------|------|
| `planner` | `NeuPanPlanner`：参考路径 + PAN |
| `obstacle_extractor` | Scan → 基座系点集 |
| `robot_model` | 运动学 + 足迹半空间 G/h |
| `distance_encoder` | ONNX DUNE 对偶 |
| `nrmp` | DAQP 二次规划 |

## 模型导出

```bash
python3 autonomy/control/controller/neupan_controller/tools/export_distance_encoder_onnx.py \
  --checkpoint autonomy/control/conf/models/neupan_model_5000.pth \
  --output autonomy/control/conf/models/neupan_distance_encoder.onnx
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
