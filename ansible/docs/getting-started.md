# 本机或远程编译

只想部署到另一台机器，使用 [部署步骤](README.md)。本页仅供需要编译安装时查阅。

## 本机编译

1. 进入宿主机 `ansible/`；缺少部署工具时入口会自动准备 Python 虚拟环境。
2. 编译、测试并安装，不启动服务：

```bash
./deploy.sh build -K \
  -e "autonomy_cmake_build_dir=$PWD/../build-ansible" \
  -e autonomy_cmake_jobs=4 \
  -e autonomy_build_test=true
```

3. 确认结尾 `failed=0`。默认安装到 `/opt/autonomy`；如果它已是制品链接，改用 `-e autonomy_install_prefix=/opt/autonomy-dev`。

## 远程编译

1. 修改 `inventory/staging/hosts.yml` 中编译机的地址和用户，保留实际主机别名。
2. 先检查，再编译（以下假设别名是 `robot01`）：

```bash
./deploy.sh check staging -l robot01
./deploy.sh build staging -l robot01 -K \
  -e autonomy_cmake_jobs=4 \
  -e autonomy_build_test=true
```

3. 确认 `failed=0`。默认源码在目标机 `/var/lib/autonomy/src/autonomy`，构建目录为其 `build/`，安装到 `/opt/autonomy`。远程构建拉取仓库，不会同步控制机未提交的修改。

staging 默认要求 Ubuntu 22.04。需要固定源码版本时增加 `-e autonomy_repo_version=YOUR_APPROVED_40_CHARACTER_COMMIT`，将占位值替换为实际完整提交。

## 已有依赖时跳过安装

在 build 命令末尾增加以下参数，避免安装依赖及 ccache：

```bash
-e autonomy_install_dependencies=false -e autonomy_use_ccache=false
```

该行是参数片段，不是独立命令。其他构建参数见 [defaults](../roles/deployment_defaults/defaults/main.yml)。
