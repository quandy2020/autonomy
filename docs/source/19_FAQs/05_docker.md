# 5. Docker 问题

常见症状：容器起不来、挂载空目录、容器内找不到数据集或连不上 Bridge/GPU。

宿主 **`AUTONOMY_ENV`** 指向含源码的树；数据集用 **`AUTONOMY_DATASETS`**。Bridge 在宿主机时需映射端口或 host 网络；与 BUILD_AUTOVIZ 无直接关系。

## 问答

### Q: 无法启动容器？

```bash
sudo systemctl start docker && docker --version
```

用户需在 `docker` 组，或按 Docker 文档 rootless。

### Q: 容器里 /workspace/autonomy 是空的？

```bash
export AUTONOMY_ENV=/path/to/host/repo
python3 docker/run_autonomy.py -p x86_64
```

### Q: docker exec 进不去？

```bash
docker ps && docker exec -it SpaceHero /bin/bash
```

容器名以 `run_autonomy.py` 输出为准。

### Q: 数据集路径不存在？

```bash
export AUTONOMY_DATASETS=/path/to/datasets
AUTONOMY_FORCE_RECREATE=1 python3 docker/run_autonomy.py
```

见 [18 · 数据集](../18_Tools/08_datasets.md)。

### Q: 容器内 grpcurl 连 Bridge？

Bridge 在宿主机：映射 `5005` 或 host 网络；在容器内 launch 则用 `127.0.0.1:5005`。

### Q: GPU 不可用？

安装 NVIDIA Container Toolkit；镜像需 cuda 变体。

### Q: 生成文件属主 root？

```bash
python3 docker/run_autonomy.py --as-host-user
```

### Q: 核对挂载？

```bash
docker exec SpaceHero ls /workspace/autonomy /data
docker exec SpaceHero printenv AUTONOMY_DATASETS
```
