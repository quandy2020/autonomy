# 5. Docker 问题

### Q: 无法启动容器？

```bash
sudo systemctl start docker
docker --version
```

安装 Docker：见 [02 Installation · Docker](../02_Installation/05_docker.md)。

### Q: 容器内找不到代码或目录为空？

检查 `AUTONOMY_ENV` 是否指向正确的仓库根路径：

```bash
export AUTONOMY_ENV=/path/to/autonomy
python3 docker/run_autonomy.py -p x86_64
```

### Q: `docker exec` 失败？

```bash
docker ps
# 检查 AUTONOMY_CONTAINER_NAME，默认常为 SpaceHero
docker exec -it SpaceHero /bin/bash
```

### Q: 数据盘不可写？

使用数据卷挂载：

```bash
python3 docker/run_autonomy.py --data-volume /mnt/data4t
```

### Q: GPU 在容器内不可用？

安装 NVIDIA Container Toolkit，见 `docker/install/install_nvidia_docker.sh`。

### Q: 权限 denied（root 创建的文件）？

```bash
python3 docker/run_autonomy.py --as-host-user
```

### Q: 如何验证挂载？

```bash
docker exec -it SpaceHero ls /workspace/autonomy
docker exec -it SpaceHero ls /mnt/data4t
```
