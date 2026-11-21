# 本机或远程编译

新工作区先执行 `git submodule update --init --recursive`。从仓库根目录运行 `bash ansible/scripts/doctor.sh native` 检查本机构建工具和子模块，控制机交叉编译时用 `bash ansible/scripts/doctor.sh cross`。

若 Git 报 `not our ref`，先确认子模块远端已发布主仓锁定的提交，再重试递归检出。

只想部署到另一台机器，使用 [部署步骤](README.md)。本页仅供需要编译安装时查阅。

## 构建前代码规范

在 `ansible/` 执行 `./deploy.sh style`，也可以传入仓库相对路径，例如 `./deploy.sh style autonomy/task`。门禁遵循仓库各级 `.clang-format` 中基于 Google 的规则，检查第一方 Git 源码和新文件，排除第三方、构建目录和生成的 Protobuf 源文件。控制台打印首批诊断、总数和完整报告路径。可用 `./deploy.sh style --report /tmp/autonomy-style.log` 自选报告路径。任何警告或错误都会返回非零，源码保持原样。

从仓库根目录运行 `python3 tools/python/clang_format_sources.py path/to/file.cpp` 可修复指定文件的格式。实际开发机、远端编译机或原生构建容器需要安装 `clang-format`。修完重新运行构建入口即可。格式门禁覆盖 `clang-format` 的诊断，CMake 编译和 CTest 仍负责各自的检查。

检查缓存只复用已通过且源码、规则和格式化器均未变化的结果。日志打印实际检查数、缓存命中数和耗时。用 `AUTONOMY_STYLE_CACHE=0` 强制重新检查。需要统一版本时，在执行检查的构建环境设置 `AUTONOMY_CLANG_FORMAT` 和 `AUTONOMY_CLANG_FORMAT_VERSION`。发布工作流可从同名 GitHub 仓库变量读取预期版本。

## 只编译指定模块

在 `ansible/` 中运行 `./deploy.sh build --module autodriver --module autonomy/transform`。也可在 `build` 后指定 inventory，并用 `-l` 选取编译主机。命令执行预检和格式门禁，自动编译依赖，打印独立的产物目录，不执行应用安装。

原生构建加 `--test` 会运行这次 CMake 配置注册的 CTest 用例。测试失败或没有注册测试时返回非零。交叉目标的运行验证在 staging 板端执行。

直接用 CMake 时，从仓库根目录先运行 `./ansible/deploy.sh style`，通过后执行 `cmake -S . -B build/selected '-DAUTONOMY_BUILD_MODULES=autolink;autonomy/transform'`，再执行 `cmake --build build/selected --parallel`。

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
