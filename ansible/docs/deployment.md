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

### 增量分发

给 `deploy`、`release` 或 `cross-release` 加 `--incremental`。已有制品用 `deploy`，需要增量交叉编译并部署时用下面的命令。

```bash
./deploy.sh cross-release robots -l robot01 -K --incremental
```

目标会复用当前版本中内容相同的文件，只接收变动文件及归档元数据。首次安装、找不到当前受管版本或差分包不比分发完整包更小时，自动传完整包。增量打包时，如果安装内容和制品元数据相同，归档摘要也相同，已安装的版本可直接复用。

目标仍会生成完整的新版本目录，切换版本时照常重启并检查服务，保留旧版供回滚。可在 inventory 用 `autonomy_artifact_incremental: true` 启用增量传输。仅需编译、打包时使用 `./deploy.sh cross-build --incremental`。现场 `build` 和原生 `release` 已复用各自的构建目录。

## 回滚到旧版本

1. 在宿主机 `~/.autonomy/artifacts/` 放入已验收旧版本的 `autonomy.tar.gz` 和对应 `.sha256`，先保留当前制品原件。
2. 使用旧制品直接部署，不使用会重新编译的 release：

```bash
artifact_dir="$HOME/.autonomy/artifacts"
(cd "$artifact_dir" && sha256sum --check autonomy.tar.gz.sha256)
artifact_sha=$(cut -d ' ' -f 1 "$artifact_dir/autonomy.tar.gz.sha256")
./deploy.sh deploy robots -l robot01 -K \
  -e "autonomy_artifact_path=$artifact_dir/autonomy.tar.gz" \
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

程序日志默认写入运行账号的 `~/.autonomy/log`，目录自动创建；本机 `quandy` 用户对应 `/home/quandy/.autonomy/log`，默认 systemd 服务账号对应 `/var/lib/autonomy/.autonomy/log`。临时改目录可使用 `GLOG_log_dir=/path/to/log /opt/autonomy/bin/autolink node list`；部署服务通过 `autonomy_log_dir` 配置。控制台输出仍可通过终端或 `journalctl` 查看，已有日志不会自动迁移。

## 使用 Autolink 命令

默认发布配置启用 `BUILD_TOOLS=ON`，同时安装 Autolink 运行库和命令行工具。在目标机执行：

```bash
/opt/autonomy/bin/autolink --help
```

需要直接输入 `autolink` 时，先在当前 Bash 终端加载环境：

```bash
source /opt/autonomy/share/autonomy/environment/library_path.sh
autolink node list
```

用户目录安装时，将 `/opt/autonomy` 替换为实际安装目录。

服务选择变化后需完整部署，push/restart 不修改所选服务集合。生产发布与回滚走 [CI/CD](ci-cd.md)。
