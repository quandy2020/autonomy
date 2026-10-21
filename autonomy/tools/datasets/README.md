# datasets

离线序列 → autolink 话题（Argus 订阅）。

**文档（正文）：** [docs/18_Tools/08_datasets.md](../../../docs/source/18_Tools/08_datasets.md)

```bash
# 宿主机
export AUTONOMY_DATASETS=/path/to/datasets
AUTONOMY_FORCE_RECREATE=1 python3 docker/run_autonomy.py

# 容器（示例序列；列表：ls /data/Oxford/sequences）
export LD_LIBRARY_PATH="/workspace/autonomy/install/autonomy/lib:/workspace/autonomy/build/autonomy/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="/workspace/autonomy/src/autonomy:/workspace/autonomy/install/autonomy/python:/workspace/autonomy/install/autonomy/lib/python:${PYTHONPATH:-}"
/usr/bin/python3 -m autonomy.tools.datasets publish \
  --dataset spires \
  --path /data/Oxford/sequences/2024-03-12-keble-college-02 \
  --rate 1.0
# 默认同时发 GT：/gt/odometry + /gt/path
# 关闭：--no-gt ；指定：--gt-file .../processed/vilens-slam/gt.tum
```

| 话题 | 说明 |
|------|------|
| `/imu` `/points` `/camera/rgb/image_raw` | 传感 |
| `/gt/odometry` | TUM 真值，按雷达时刻最近邻 |
| `/gt/path` | 随播放时刻增长的 GT 轨迹（与 `/argus/trajectory` 同步拉长） |

GT 默认优先 `processed/vilens-slam/gt.tum`（与 Argus 评测一致），否则 `processed/trajectory/gt-tum.txt`。

| 模块 | 职责 |
|------|------|
| `cli.py` | `info` / `publish` |
| `publisher.py` | 时序发布 |
| `tum.py` | TUM 真值 |
| `readers/spires.py` | Oxford Spires |
| `pcd_io.py` / `messages.py` | PCD 与消息编码 |
