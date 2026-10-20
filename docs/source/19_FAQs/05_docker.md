# 5. Docker 问题

### Q: 无法启动容器？

```bash
sudo systemctl start docker && docker --version
```

### Q: 容器内代码为空？

```bash
export AUTONOMY_ENV=/path/to/autonomy
python3 docker/run_autonomy.py -p x86_64
```

### Q: docker exec 失败？

```bash
docker ps && docker exec -it SpaceHero /bin/bash
```

### Q: 数据盘不可写？

```bash
python3 docker/run_autonomy.py --data-volume /mnt/data4t
```

### Q: 没有 `/data/Oxford/...`？

```bash
export AUTONOMY_DATASETS=/path/to/datasets
AUTONOMY_FORCE_RECREATE=1 python3 docker/run_autonomy.py
```

见 [18 · §8](../18_Tools/08_datasets.md)。

### Q: GPU 不可用？

安装 NVIDIA Container Toolkit。

### Q: 属主为 root？

```bash
python3 docker/run_autonomy.py --as-host-user
```

### Q: 核对挂载？

```bash
docker exec SpaceHero ls /workspace/autonomy /data
docker exec SpaceHero printenv AUTONOMY_DATASETS
```
