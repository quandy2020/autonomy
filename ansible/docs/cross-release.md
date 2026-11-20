# 交叉编译发布

`cross-build` 在控制机用共享 Docker 镜像编译并生成制品。`cross-release` 先执行同一构建，再调用现有 Ansible 制品部署流程。一条命令对应一个 Profile，可同时选择该 Profile 的多台机器人。

## 准备一次

1. 从仓库根目录运行 `python3 docker/build_docker_x86_64.py`，或准备带目标工具链的兼容镜像。
2. 编辑 `ansible/vars/release-cross.yml`，选择目标 Profile。首次获取或更新 sysroot 时填写参考板的 `sysroot.host` 和 `sysroot.user`。复用已有 sysroot 时省略整个 `sysroot` 字段。
3. 在目标的 inventory 或 group_vars 中填写 `autonomy_cross_profile`。同一批目标使用相同 Profile。镜像内需有与 sysroot 中目标库匹配的宿主 `protoc`，启用 gRPC 时还需要宿主 `grpc_cpp_plugin`。目标机预先安装 Autonomy 所需的运行库和 SDK。

例如在实际机器人组中配置：

```yaml
autonomy_cross_profile: rockchip-rk3588-ubuntu2204
```

## 一键命令

从 `ansible/` 执行：

```sh
# 增量交叉编译并打包。输出制品路径和校验值。
./deploy.sh cross-build --incremental

# 增量编译一次，向选中的机器人增量分发。
./deploy.sh cross-release robots -l robot01 -K --incremental

# 向一组同 Profile 的机器人分发。
./deploy.sh cross-release robots -l rk3588_robots -K --incremental
```

首次构建没有缓存时会编译全部内容。后续由 Ninja 只重编受影响的目标。缓存位于 `tools/cross-build/work/incremental/<profile>/`，同一源码目录可跨提交复用。镜像、sysroot 内容、工具链或 CMake 参数变化时会启用新缓存。每次发布使用新的安装目录，省略 `--incremental` 可强制全新交叉构建。

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

`cross-release` 支持 robots 与 staging，遵守环境已有预检和滚动部署策略。RK3506 Debian 12 使用 robots，默认 staging 仍要求 Ubuntu 22.04。生产环境通过已有审批和制品晋级流程部署。

## 实现与验收

控制机复用 cross-build 的 Profile、sysroot 和 CMake 工具链。交叉制品记录目标 Profile、系统版本、ABI 和源码提交，打包时检查目标 ELF。发布包包含 CMake 安装产物和组件配置，第三方运行库由目标系统提供。镜像内有多套 Protobuf 工具时，在 `build_args` 中用 `-DAUTONOMY_HOST_PROTOC=...` 选定宿主版本。

待验收顺序为控制机离线回归、两种 ABI 的真实构建及 ELF 核对、同 Profile 的 staging 或机器人部署与服务检查。使用 [cross-build 快速上手](../../tools/cross-build/README.md) 准备开发环境。
