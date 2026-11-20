# 遇到错误时怎么处理

只处理出错的那一步，成功后再继续 [部署步骤](README.md)。分享日志前先脱敏，不输出私钥、密码或 vault 内容。

## 工具或 Collection 找不到

`deploy.sh` 缺少工具时会自动准备 `.venv`，失败时先检查宿主机 Python 3.12、venv 支持和安装源网络。直接调用 Ansible 时先执行 `source .venv/bin/activate` 和 `./scripts/install_collection.sh`。

## SSH 或 sudo 失败

核对 `inventory/robots/hosts.yml` 的 IP、用户，以及 SSH 密钥和网络。先用相同用户普通 SSH 登录，核实 host key；不要关闭 host key 校验。需要 sudo 密码时使用 `-K`，CI 则配置免交互 sudo。

## 系统、架构或依赖不匹配

目标机必须与制品的 OS、CPU 架构和运行依赖基线一致。staging/production 当前严格要求 Ubuntu 22.04。缺少动态库时补齐批准的平台依赖，或在匹配基线上重新构建；不要混用其他机器的二进制。

## 一键编译或打包失败

检查 `vars/release.yml` 的容器名、容器内路径和 build_args；确认容器运行且编译依赖齐全。release 默认不向目标机安装依赖，目标须已有匹配的运行基线。准备制品失败时不会进入部署，不要手工使用不明版本的旧制品继续发布。

## CMake 缓存冲突

改用独立的 `autonomy_cmake_build_dir`，见 [编译步骤](getting-started.md)。不要直接删除 colcon 或其他工作的构建目录。安装前缀若是 release 链接，改用独立机器或 `/opt/autonomy-dev`。

## checksum 或二进制检查失败

确认 tarball 和 `.sha256` 是同一版本，再重新打包/传输。默认选择 bridge，需要 `BUILD_GRPC=ON` 和 `AUTONOMY_BUILD_BRIDGE=ON`；occupancy_grid 还需要它自己的可执行文件。不要跳过校验。

## 服务启动失败

SSH 登录目标机执行：

```bash
systemctl status autonomy-task --no-pager
journalctl -u autonomy-task -n 100 --no-pager
readlink -f /opt/autonomy
```

按实际失败服务替换名称，核对配置权限、二进制和动态库。失败可能发生在版本已切换之后，不能假定旧版本仍在运行。push 无变化时不重启；需要强制加载配置时执行 `./deploy.sh restart robots -l robot01 -K`。

## production 发布被拒绝

按 [CI/CD](ci-cd.md) 使用成功 staging run、相同提交和架构。审计存在不等于发布成功。锁定依赖安装需要精确 APT pins，runner、网络、审批和 secrets 也必须已配置；不要关闭生产检查来绕过失败。
