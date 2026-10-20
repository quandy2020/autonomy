# 10. Perception Base

目录：`autonomy/perception/base/`。共享一套 RGB 加载器，具体任务在 `tasks/<name>/`。权重放 `config/perception/base/`，不进 git。配置用 `proto/base.proto` 里的 `repeated tasks`。

| 任务 | `kind` | 话题 |
|------|--------|------|
| 检测 | `TASK_DETECT` | `/perception/base/detections` |
| 分割 | `TASK_SEGMENT` | `/perception/base/masks` |
| 分类 | `TASK_CLASSIFY` | `/perception/base/classification` |
| 姿态 | `TASK_POSE` | `/perception/base/poses` |
| 旋转框 | `TASK_OBB` | `/perception/base/obb` |
| 跟踪 | `TASK_TRACK` | `/perception/base/tracks` |
| 深度 | `TASK_DEPTH` | `/perception/base/depth` |

深度用 MoGe，样例配置 `config/perception/base/base_depth_moge.pb.txt`。输入 `image [1,3,H,W]`、RGB 归一化到 `[0,1]`，深度大致是 `(points.z + shift) * metric_scale`。

```bash
XFORMERS_DISABLED=1 python config/perception/base/export_moge_trt.py \
  --pretrained config/perception/base/moge_ckpt/model.pt
```

Detect 导出约定：`output0 [1,max_det,6]` = `x1,y1,x2,y2,score,class`。Track 在检测结果上写稳定 `id`。除 MoGe 外，YOLO head 的 decode 绑定还在补，开任务前先确认 engine 已导出。更多细节直接看源码 `base/README.md`。
