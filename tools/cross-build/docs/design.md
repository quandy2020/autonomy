# cross-build 实现设计

## 目标与边界

在本目录统一管理不同厂商、芯片、目标系统的 Linux 用户态 C/C++ 交叉编译环境。提供可重复调用的命令，从 Profile 选择、sysroot 获取和校验，到容器内 CMake 编译和安装。仓库 `docker/` 负责构建共享开发镜像。

首批 Profile：

| Profile ID | 目标 ABI | 容器工具链基线 |
| --- | --- | --- |
| rockchip-rk3588-ubuntu2204 | aarch64-linux-gnu | 共享镜像 / GCC 11 |
| rockchip-rk3506-debian12 | arm-linux-gnueabihf | 共享镜像 / GCC 12 |
| nvidia-agx-orin-jetpack6 | aarch64-linux-gnu | 共享镜像 / GCC 11，普通 C/C++ |
| nvidia-orin-nx-jetpack6 | aarch64-linux-gnu | 同上 |
| nvidia-orin-nano-jetpack6 | aarch64-linux-gnu | 同上 |

上述工具链是拟采用的用户态构建基线，不声称是 BSP 官方工具链或板端实际版本。RK3506 Profile 假定目标运行 Debian 12 hard-float glibc；其他 ABI 的厂商固件不能直接复用。JetPack 6 只是 Profile 系列名，实际 JetPack/L4T、CUDA 和目标 libc 版本应记录在 sysroot 清单中。没有板端信息时不能声称验证了部署兼容性。

## 方案选择

1. 每个平台复制 Dockerfile、脚本：上手快，但流程和校验易漂移。
2. **采用：结构化 Profile、共用 CMake 工具链和仓库已有开发镜像。** 芯片参数和目标 sysroot 由 Profile 选择。共享镜像预装两组编译器，五个平台共用执行路径。
3. 通过 QEMU 运行目标架构容器进行本机构建：可用于特殊场景，但速度和执行模型不符合本次交叉编译目标。

使用 Python 标准库承担命令编排及校验，Shell 只作为入口。profile.yaml 使用 JSON 语法（YAML 1.2 子集），避免只为读取配置增加依赖；文档明确编辑格式。保留标准字段，不通过拆分目录名推导元数据。

## 命名与元数据

Profile ID 为 `{vendor}-{model}-{stack}{version}`，对应五个已给定名称。version 为 ID 中的纯数字标记；另外保留真实软件版本字符串，避免用任意删除标点的规则造成版本碰撞。校验目录名、platform 字段与结构化拼接结果一致，检查字符集、必填字段、架构与编译器 triple、平台文件路径。

平台文件按 `{vendor}-{family}.cmake` 命名，三个 Orin Profile 共用 nvidia-orin.cmake。把空目录 scritps 更正为 scripts；没有现有调用方，不建立兼容别名。

## 操作流程

- `make list`：列出 Profile 与目标 ABI。
- `make check`：离线校验全部 Profile、CMake include 目标及配置。
- `make sysroot PROFILE=... HOST=... USER=...`：通过 SSH/rsync 获取目标 `/lib`、`/usr/lib`、`/usr/include`、系统版本信息；可选显式目录获取 SDK。不默认拷贝整个 `/usr/local`。
- `make verify PROFILE=...`：检查 sysroot 头文件、启动文件、libc ELF 位数/机器类型、关键符号链接、系统版本；失败返回非零。
- 共享镜像由 `docker/build_docker_x86_64.py` 构建。它包含 AArch64 GCC 11、armhf GCC 12、CMake、Ninja 和 pkg-config。Apple Silicon 使用默认的 amd64 镜像需要 Docker 仿真支持。
- `make build PROFILE=... SOURCE=...`：先校验 Profile 和 sysroot，再运行容器，挂载只读源码与 sysroot、独立可写构建及安装目录，传入工具链配置 CMake 并编译。支持参数透传及 install 入口。

每个 Profile 提供 toolchain.cmake，加载共用 Linux 工具链及芯片文件。设置编译器、sysroot、CMake 查找根和目标 pkg-config 路径，隔离宿主库。RK3588 使用兼容其大小核的 ARMv8 基线；RK3506 使用 Cortex-A7 hard-float 参数；Orin 使用 ARMv8 基线，CUDA 显式启用时设置架构 87。不默认使用 static-library try_compile 掩盖链接失败。

## sysroot 安全与可追溯性

仅向本目录忽略的工作区写入 sysroot，不同步到用户任意目录、不使用 rsync --delete。先同步到临时目录，校验通过后才替换已有 sysroot，失败保留旧版本。拒绝越界路径；转换绝对符号链接时保留其 sysroot 内语义，发现越界或关键断链时报错，不访问宿主对应路径。记录 Profile、来源、采集时间、目标系统版本，编译器真实版本在构建日志中输出。这里不承诺二进制级可重复构建，发行版包更新会影响镜像，正式发布需固定基础镜像摘要及包源快照。

## SDK 与依赖边界

基础镜像不自动下载或分发 CUDA、cuDNN、TensorRT、RKNN、MPP、RGA。目标 SDK 的头文件和库来自匹配目标的 sysroot 或显式 SDK 路径；CUDA 编译另需匹配版本的宿主 nvcc 及目标库，不能直接运行从板端复制的 ARM nvcc。文档说明使用 NVIDIA 官方交叉编译镜像的接入方式与限制，不声称基础镜像已支持 CUDA 编译。交叉构建 Protobuf 等项目时，宿主代码生成工具需单独提供。

## 文件范围

- profiles/*/profile.yaml、toolchain.cmake：五套元数据和入口。
- cmake/linux.cmake、cmake/platform/*.cmake：共用工具链和芯片参数。
- scripts/：元数据读取、sysroot 获取及校验、容器构建入口。
- Makefile、.gitignore。共享镜像的定义和构建入口位于仓库的 docker/dockerfile/autonomy.x86_64.dockerfile 与 docker/build_docker_x86_64.py。
- tests/：元数据、ELF/符号链接校验、失败保留旧 sysroot、命令参数与路径隔离测试；最小 C/C++ smoke 工程。
- docs/profiles.md、onboarding.md、troubleshooting.md 和 README.md。
- ci/gitlab-ci.yml：可 include 的五 Profile 校验/构建示例，不擅自接入现有 CI；真实编译作业要求 runner 预置对应 sysroot。

## 验收与限制

离线测试必须覆盖非法 Profile、错误 ELF/ABI、越界符号链接和执行失败；所有 Profile 通过元数据校验；Shell、Python、CMake 入口通过语法检查；可用时用 Docker 对两种 ABI 编译最小 C/C++ 程序并检查 ELF。Docker、网络、板端 sysroot 或真实设备不可用时明确列出未执行项，不能以 mock 通过代替真实交叉编译或部署验收。

不新增全局 CI、自动刷机、远程执行产物、SDK 下载器或整仓库依赖打包。

## 实现阶段补充

用户已要求直接完成设计、编码和文档，所有验证待后续通知。实现采用单 Profile 目录锁，保留 previous-UUID 备份并在发布失败时恢复；不宣称跨崩溃原子性。SOURCE 默认仓库根目录，CMake 参数通过 CLI 的 -- 原样透传，容器使用当前 UID/GID。只读源码兼容性及厂商镜像适配要求见 onboarding.md。
