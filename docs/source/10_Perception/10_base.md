# 10. Perception Base

出 `/perception/base/*` 检测/跟踪/深度话题，或导出 MoGe/YOLO engine。

目录 `autonomy/perception/base/`：共享 RGB 加载，任务在 `tasks/<name>/`。配置 `proto/base.proto` 的 `repeated tasks`；权重在 `config/perception/base/`（不进 git）。

| 任务 | `kind` | 话题 |
|------|--------|------|
| 检测 | `TASK_DETECT` | `/perception/base/detections` |
| 分割 | `TASK_SEGMENT` | `/perception/base/masks` |
| 分类 | `TASK_CLASSIFY` | `/perception/base/classification` |
| 姿态 | `TASK_POSE` | `/perception/base/poses` |
| 旋转框 | `TASK_OBB` | `/perception/base/obb` |
| 跟踪 | `TASK_TRACK` | `/perception/base/tracks` |
| 深度 | `TASK_DEPTH` | `/perception/base/depth` |

MoGe 样例：`config/perception/base/base_depth_moge.pb.txt`。深度尺度：$(points.z + shift) * metric_scale$。

```bash
XFORMERS_DISABLED=1 python config/perception/base/export_moge_trt.py \
  --pretrained config/perception/base/moge_ckpt/model.pt
```

Detect 输出约定：`output0 [1,max_det,6]` = x1,y1,x2,y2,score,class。

- 与 **costmap** 无自动连接；跟 MPPI 见 [§11 Follow](11_follow.md)。  
- 除 MoGe 外，部分 YOLO decode 仍在补；开任务前确认 engine 已导出。  
- 启动用 base DAG/launch，**不是** `PerceptionServer`。

### 常见坑

| 现象 | 先查 |
|------|------|
| 无输出 | 模型路径、task kind、相机 topic |
| 与 network 文档混读 | Base 已包业务；network 是底层引擎 [§7](07_vision_detection.md) |

细节：`base/README.md`。
