# 交叉编译发布

`cross-build` 在控制机用共享 Docker 镜像编译并生成制品。`cross-release` 先执行同一构建，再调用现有 Ansible 制品部署流程。一条命令对应一个 Profile，可同时选择该 Profile 的多台机器人。

## 准备一次

1. 从仓库根目录运行 `python3 docker/build_docker_x86_64.py`，或准备带目标工具链的兼容镜像。
2. 编辑 `ansible/vars/release-cross.yml`，选择目标 Profile。填写 `sysroot.host` 和 `sysroot.user` 后，首次使用或参考板设置改变时才重新采集。板端 SDK 更新时可用 `sysroot.refresh: true` 显式刷新，完成后恢复为 `false`。直接使用预置 sysroot 时省略整个 `sysroot` 字段。
3. 在目标的 inventory 或 group_vars 中填写 `autonomy_cross_profile`。同一批目标使用相同 Profile。镜像内需有与 sysroot 中目标库匹配的宿主 `protoc`，启用 gRPC 时还需要宿主 `grpc_cpp_plugin`。目标机预先安装 Autonomy 所需的运行库和 SDK。

例如在实际机器人组中配置：

```yaml
autonomy_cross_profile: rockchip-rk3588-ubuntu2204
```

## 一键命令

从 `ansible/` 执行：

```sh
# 检查 inventory、SSH、目标 OS、ABI 和磁盘。也会在 cross-release 内自动执行。
./deploy.sh plan robots -l robot01 --release-config vars/release-cross.yml

# 增量交叉编译并打包。输出制品路径和校验值。
./deploy.sh cross-build --incremental

# 只交叉编译所选模块，自动带上依赖。输出构建目录。
./deploy.sh cross-build --incremental --module automsgs --module autonomy/control

# 增量编译一次，向选中的机器人增量分发。
./deploy.sh cross-release robots -l robot01 -K --incremental

# 向一组同 Profile 的机器人分发。
./deploy.sh cross-release robots -l rk3588_robots -K --incremental

# 单独复核当前安装和服务，不重启。
./deploy.sh verify robots -l robot01
```

首次构建没有缓存时会编译全部内容。后续由 Ninja 只重编受影响的目标。缓存位于 `tools/cross-build/work/incremental/<profile>/`，同一源码目录可跨提交复用。镜像、sysroot 内容、工具链或 CMake 参数变化时会启用新缓存。控制机采集的 sysroot 在文件元数据一致时复用内容摘要，用 `CROSS_BUILD_REHASH_SYSROOT=1` 可强制重新计算。每次发布使用新的安装目录，省略 `--incremental` 可强制全新交叉构建。

`--module` 可重复。现场 `build` 以及未指定基线的 `cross-build` 只编译，产物留在独立的构建目录。

## 增量部署指定模块

先把 `cross-build --incremental` 生成的完整制品部署并验收，保留该归档。目标机需配置 `autonomy_health_probe_argv`。用已部署归档的绝对路径作为基线：

```sh
base=/absolute/path/to/current-autonomy.tar.gz
./deploy.sh plan robots -l robot01 -K \
  --base-artifact "$base" --module automsgs --module autonomy/control
./deploy.sh cross-release robots -l robot01 -K \
  --base-artifact "$base" --module automsgs --module autonomy/control
```

`cross-release` 自动执行同一预检，先核对活动版本、原基线、健康检查点和服务集合，再开始编译。系统安装需要 sudo 读取受保护的检查点。发布锁内再次核对活动版本。命令自动启用增量编译和增量传输，将模块运行文件合入完整制品，保留现场配置和 systemd 单元。验证失败时恢复旧版本并复检。下一次更新可把本次成功发布的归档作为基线。

先生成制品时，把 `cross-release robots …` 换成 `cross-build --base-artifact "$base" --module …`，再按下文的 `deploy` 用法分发。模块发布适用于 `robots` 和 `staging`。基线与模块构建的 Profile、sysroot 和镜像必须一致。需要删除旧文件、更新现场配置或引入不兼容 ABI 时，执行完整发布。

默认制品目录为 `~/.autonomy/artifacts/`。用 `--release-config FILE` 切换 Profile 配置。控制机按制品摘要命名归档并生成 `.sha256` 文件，构建失败时部署阶段不会开始。

已有归档可直接分发。将下方路径替换为 `cross-build` 输出中的 `autonomy_artifact_path`。

```sh
archive=/absolute/path/to/autonomy-artifact.tar.gz
./deploy.sh deploy robots -l robot01 -K \
  -e "autonomy_artifact_path=$archive" \
  -e "autonomy_artifact_sha256=$(awk '{print $1}' "$archive.sha256")"
```

如果制品配置了自定义 `install_prefix`，再加 `-e autonomy_install_prefix=/your/prefix`。

已有归档的 `deploy` 命令也支持 `--incremental`。完整版本及回滚用法见 [日常部署](deployment.md#增量分发)。

`cross-release` 支持 robots 与 staging，默认逐台验证再继续。RK3506 主机需明确声明 Debian Profile，部署时会核对 Debian 12 和 armhf。生产环境通过审批后晋级同一份 staging 制品。

## 实现与验收

控制机复用 cross-build 的 Profile、sysroot 和 CMake 工具链。交叉制品记录目标 Profile、系统版本、ABI 和源码提交，打包时检查目标 ELF。发布包包含 CMake 安装产物和组件配置，第三方运行库由目标系统提供。镜像内有多套 Protobuf 工具时，在 `build_args` 中用 `-DAUTONOMY_HOST_PROTOC=...` 选定宿主版本。

在 staging 分别用一台 `aarch64` 和一台 `armhf` 验收。每次 `-l` 只选一台，记录 Profile、旧版与新版 SHA-256 和以下结果。

1. 完整构建及目标 ELF 校验通过，完整部署后执行 `verify`。
2. 修改一个模块，运行基线预检和增量发布，记录格式、配置、编译、打包、传输及探针耗时，以及传输字节数。
3. 重复部署同一制品，确认活动版本不变。用错误基线运行 `plan`，确认编译未启动。
4. 在验收目标上用可恢复的故障夹具触发探针失败，确认旧版本与现场配置恢复、`verify` 通过且锁释放。使用 Autoviz 的配置另验收与 task/localization 联跑和退出。

开发环境见 [cross-build 快速上手](../../tools/cross-build/README.md)。
