# cross-build 快速上手

用仓库的共享开发镜像交叉编译 Linux C/C++ 工程。Profile 选择目标平台，sysroot 提供板端头文件和库。

宿主需要 Docker、Python 3.9+、GNU Make、SSH 和支持 `--protect-args` 的 rsync。板端需要 rsync、可通过 SSH 读取的系统文件，以及与固件匹配的 C/C++ 开发包。

## 1. 选择 Profile

| 目标板与系统 | Profile | 目标 ABI |
| --- | --- | --- |
| RK3588 · Ubuntu 22.04 | `rockchip-rk3588-ubuntu2204` | AArch64 |
| RK3506 · Debian 12 | `rockchip-rk3506-debian12` | ARMv7 hard-float |
| Jetson AGX Orin · JetPack 6 | `nvidia-agx-orin-jetpack6` | AArch64 |
| Jetson Orin NX · JetPack 6 | `nvidia-orin-nx-jetpack6` | AArch64 |
| Jetson Orin Nano · JetPack 6 | `nvidia-orin-nano-jetpack6` | AArch64 |

## 2. 准备共享镜像

首次使用时，从仓库根目录运行一次。已有包含目标编译器的 `autonomy.platform.x86_64` 镜像可直接复用。Apple Silicon 使用默认镜像需要 Docker 的 x86 仿真支持。

```sh
python3 docker/build_docker_x86_64.py
```

## 3. 采集 sysroot，编译示例

在仓库根目录执行。把示例里的 Profile、IP 和 SSH 用户改为你的板卡信息。

```sh
cd tools/cross-build
profile=rockchip-rk3588-ubuntu2204
make sysroot PROFILE="$profile" HOST=192.168.1.100 USER=robot
make install PROFILE="$profile" SOURCE="$PWD/tests/smoke"
```

采集结果在 `work/sysroots/<profile>/`，示例程序安装到 `work/install/<profile>/bin/`。只编译时使用 `make build`，只检查已有 sysroot 时使用 `make verify PROFILE=...`。

## 4. 编译自己的工程

把 `SOURCE` 指向含有 `CMakeLists.txt` 的工程目录。省略 `SOURCE` 时使用 Autonomy 仓库根目录。业务工程需在 sysroot 中准备目标依赖，并在镜像中准备匹配的宿主代码生成工具。

同一 Profile 切换源码工程前，将 `work/build/<profile>/` 和 `work/install/<profile>/` 移出原位，避免沿用 smoke 的缓存和产物。更换 sysroot 或镜像时也建议从干净的构建目录开始。

```sh
make install PROFILE="$profile" SOURCE=/absolute/path/to/project
```

## 5. 按模块编译 Autonomy

从 `tools/cross-build/` 执行。`MODULES` 用空格分隔，命令行的 `--module` 可重复。

```sh
make build PROFILE="$profile" MODULES="automsgs autonomy/control"
python3 scripts/cross_build.py build --profile "$profile" \
  --module automsgs --module autonomy/control --incremental
```

支持 `automsgs`、`autolink`、`autodriver`、`autoviz`、`automanip` 和 `autonomy`。`autonomy` 选择启用的领域，也可指定 `autonomy/common`、`autonomy/control`、`autonomy/localization` 等单个领域。CMake 自动加入依赖，并在配置时列出项目和领域。比如 `autonomy/control` 同时编译 `common`、`transform`、`map` 和共享 Protobuf 库。

同一组模块可复用自己的构建目录，路径在构建结束后打印。此入口只编译，仍执行全仓第一方 C/C++ 格式门禁。`autoviz` 需目标 Qt 和 Ogre SDK，显式选择 `autonomy/bridge` 需启用 `BUILD_GRPC`。需要发布所选模块时，使用 [Ansible 模块发布](../../ansible/docs/cross-release.md#增量部署指定模块)。

- [自定义镜像、CMake 参数和厂商 SDK](docs/onboarding.md)
- [用 Ansible 一键编译、打包和部署](../../ansible/docs/cross-release.md)
- [新增 Profile](docs/profiles.md) · [故障排查](docs/troubleshooting.md) · [设计](docs/design.md)

共享镜像、交叉编译和板端运行均待验证。
