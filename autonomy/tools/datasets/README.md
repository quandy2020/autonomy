# datasets

离线序列 → autolink 话题（Argus 订阅）。

**文档（正文）：** [docs/18_Tools/09_datasets.md](../../../docs/source/18_Tools/09_datasets.md)

```bash
# 宿主机
export AUTONOMY_DATASETS=/path/to/datasets
AUTONOMY_FORCE_RECREATE=1 python3 docker/run_autonomy.py

# 容器
/usr/bin/python3 -m autonomy.tools.datasets publish \
  --dataset spires --path /data/Oxford/sequences/<name> --rate 1.0
```

| 模块 | 职责 |
|------|------|
| `cli.py` | `info` / `publish` |
| `publisher.py` | 时序发布 |
| `readers/spires.py` | Oxford Spires |
| `pcd_io.py` / `messages.py` | PCD 与消息编码 |
