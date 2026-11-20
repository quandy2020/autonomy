# 进阶用法

首次使用先看 [快速上手](../README.md)。以下命令从 `tools/cross-build/` 执行。

## 确认板端系统

在板端运行 `uname -m` 和 `cat /etc/os-release`。Jetson 还要查看 `/etc/nv_tegra_release`，JetPack 6 Profile 要求 L4T R36。RK3506 Profile 要求 Debian 12、glibc 和 ARM hard-float ABI。板端应安装匹配的开发包，例如 libc6-dev、libstdc++ 开发包及项目所需的目标架构依赖。

默认采集 `/lib`、`/usr/lib`、`/usr/include` 和系统版本信息。采集成功后，sysroot 内的 `.cross-build.json` 会记录板端来源、采集时间、目录和系统版本。旧 sysroot 保存在同目录下的 `<profile>.previous-<uuid>`。锁或采集失败时见 [故障排查](troubleshooting.md)。

## 传入 CMake 参数或更换镜像

```sh
python3 scripts/cross_build.py build \
  --profile rockchip-rk3588-ubuntu2204 \
  --source /absolute/path/to/project -- -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

`--` 后的参数传给 CMake 配置。想同时安装时，把 `build` 换成 `install`。源码和 sysroot 在容器内以只读方式挂载，项目需要支持在源码目录之外构建。

用 `IMAGE` 或 `--image` 选择预先准备好的兼容镜像。镜像需提供目标 triple 的 `gcc/g++`、CMake 3.20+、Ninja、pkg-config 和 `/bin/sh`。默认用 `linux/amd64`，其他镜像按实际架构设置 `DOCKER_PLATFORM` 或 `--docker-platform`。

在已安装 GNU 交叉编译器的 Linux 主机上，也可以直接调用 CMake 工具链。

```sh
profile=rockchip-rk3588-ubuntu2204
CROSS_SYSROOT="$PWD/work/sysroots/$profile" \
  cmake -S tests/smoke -B "work/native-smoke/$profile" \
  -DCMAKE_TOOLCHAIN_FILE="$PWD/profiles/$profile/toolchain.cmake"
cmake --build "work/native-smoke/$profile"
```

## 接入厂商 SDK

通过 `--extra-dir` 补采默认目录之外的目标 SDK。若路径是符号链接，也需采集链接指向的实际目录。以下 CUDA 目录仅作示例，请以板端版本为准。

```sh
python3 scripts/build_sysroot.py --profile nvidia-orin-nx-jetpack6 \
  --host 192.168.1.101 --user robot --extra-dir /usr/local/cuda-12.6
```

默认镜像提供 GNU C/C++ 工具链。启用 CUDA 时，另外准备匹配 JetPack/L4T 的宿主版 `nvcc` 和 AArch64 CUDA 库，并用 `--image` 与 `-DCMAKE_CUDA_COMPILER=...` 指定。Orin 的 CUDA 架构默认设为 87。参考 [NVIDIA 工具链说明](https://docs.nvidia.com/jetson/archives/r36.4.3/DeveloperGuide/AT/JetsonLinuxToolchain.html)和 [NVIDIA 官方交叉编译镜像](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/jetpack-linux-aarch64-crosscompile-x86)。

Autonomy 从 sysroot 查找目标 Protobuf 库。镜像中的宿主 `protoc` 必须匹配目标 Protobuf 版本，可通过 `-DAUTONOMY_HOST_PROTOC=/path/in/image/to/protoc` 指定。启用 gRPC 时，可通过 `-DGRPC_CPP_PLUGIN=...` 指定镜像内的宿主插件。两者均待真实构建验收。

## 验收清单

1. 运行 `make check`、`make test` 和 `sh -n scripts/run_build.sh`。
2. 用真实 sysroot 对 AArch64 和 armhf 分别编译并安装 smoke 工程。
3. 用目标架构的 `readelf -h -l -d -V` 检查 ELF、动态加载器、依赖库和 GLIBC/GLIBCXX 符号版本。armhf 还要检查 hard-float ABI。
4. 在对应板卡运行 smoke 产物，再验证业务工程和实际启用的 SDK。

`make verify` 检查 sysroot 结构。板端运行兼容性由第 4 步确认。
