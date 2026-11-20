# 部署后的日常操作

第一次部署按 [一键部署](README.md) 执行。以下命令均在宿主机 `ansible/` 中运行，缺少部署工具时入口会自动准备。

## 更新配置

1. 修改控制机源码中的 `autonomy/<component>/conf/`，先核对机器人标定参数。
2. 推送到指定机器：

```bash
./deploy.sh push robots -l robot01 -K
```

3. 确认 `failed=0`。有变化时自动重启，无变化不重启；同名配置覆盖，额外文件保留。

## 只重启服务

```bash
./deploy.sh restart robots -l robot01 -K
```

## 更新应用版本

执行 `./deploy.sh release robots -l robot01 -K`，重新编译、打包并部署。完整部署会重新安装制品中的配置，可能覆盖之前 push 的同名文件。

## 回滚到旧版本

1. 在宿主机 `src/autonomy/dist/` 放入已验收旧版本的 `autonomy.tar.gz` 和对应 `.sha256`，先保留当前制品原件。
2. 使用旧制品直接部署，不使用会重新编译的 release：

```bash
(cd ../dist && sha256sum --check autonomy.tar.gz.sha256)
artifact_sha=$(cut -d ' ' -f 1 ../dist/autonomy.tar.gz.sha256)
./deploy.sh deploy robots -l robot01 -K \
  -e "autonomy_artifact_path=$PWD/../dist/autonomy.tar.gz" \
  -e "autonomy_artifact_sha256=$artifact_sha"
```

3. 确认 `failed=0` 后验收目标服务和平台功能。

不要只手工切换 `/opt/autonomy.previous`：这不会恢复 `/etc/autonomy`。回滚不会删除额外配置，也不恢复地图和运行数据；失败后需要人工核对当前状态。

## 查看目标机状态

SSH 登录目标机后执行：

```bash
readlink -f /opt/autonomy
systemctl status autonomy-task --no-pager
journalctl -u autonomy-task -n 100 --no-pager
```

当前版本位于 `/opt/autonomy-releases/`，配置在 `/etc/autonomy`，运行数据在 `/var/lib/autonomy`。历史版本不会自动清理，不要删除当前或 previous 引用的目录。

服务选择变化后需完整部署，push/restart 不修改所选服务集合。生产发布与回滚走 [CI/CD](ci-cd.md)。
