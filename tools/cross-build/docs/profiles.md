# Profile 命名与扩展

## 命名规范

Profile ID：`{vendor}-{model}-{stack}{version}`，仅小写字母、数字和单个连字符。厂商全称为 rockchip、nvidia，软件栈为 ubuntu、debian、jetpack；禁止 rk、nv、jp 作为厂商或软件栈缩写。型号中的 rk 属于完整型号，例如 rk3588，必须保留。多词型号使用连字符，例如 agx-orin、orin-nx。

version 是明确给定的纯数字 ID 标记：Ubuntu 22.04 → 2204，Debian 12 → 12，JetPack 6 → 6。不要对任意版本字符串自动删除标点；例如 1.23 和 12.3 会发生碰撞。真实 OS 版本保留在 distro_version；实际 Jetson Linux 版本保存在采集清单中。

Profile ID 是结构化字段的拼接结果，不通过 split('-') 反向推导字段。`目录名 == platform == vendor + '-' + model + '-' + stack + version` 必须成立。

## 配置格式

profile.yaml **必须使用 JSON 语法**，即 YAML 1.2 的子集；不支持一般 YAML 的缩进映射、注释、锚点或未加引号的键。这样可直接使用 Python 标准库读取，避免引入 YAML 依赖。

```json
{
  "platform": "rockchip-rk3588-ubuntu2204",
  "vendor": "rockchip",
  "model": "rk3588",
  "stack": "ubuntu",
  "version": "2204",
  "arch": "aarch64",
  "family": "rk3588",
  "triple": "aarch64-linux-gnu",
  "distro": "ubuntu",
  "distro_version": "22.04",
  "gcc_major": "11"
}
```

全部字段为非空字符串。arch 支持 aarch64、armhf，分别固定 GNU triple aarch64-linux-gnu、arm-linux-gnueabihf。不要将 armhf 当成所有 32 位 ARM 固件的通用 ABI。

CMake 平台文件为 `{vendor}-{family}.cmake`：rockchip-rk3588、rockchip-rk3506、nvidia-orin。三个 Orin 模块共用芯片族配置，但各有独立 Profile、sysroot、构建目录，避免无意混用板端库。

toolchain.cmake 的规范内容如下；`check` 会与元数据生成的预期入口比较，发现陈旧引用时拒绝执行：

```cmake
set(CROSS_TRIPLE "aarch64-linux-gnu")
set(CMAKE_SYSTEM_PROCESSOR "aarch64")
include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/platform/rockchip-rk3588.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/linux.cmake")
```

所有 Profile 默认使用 `autonomy.platform.x86_64`，由仓库 `docker/` 统一构建。gcc_major 描述共享镜像中对应 ABI 的编译器基线。环境变量使用大写下划线；脚本 snake_case；目录沿用 cross-build；CI 使用 PROFILE 矩阵，避免手工拼接不同命名。

## 新增平台

1. 复制最接近的 Profile，修改结构化字段并按规则命名目录。
2. 复用相同 family 的 CMake 文件；只有硬件差异确实影响编译时才新增 family 文件。
3. 同步 toolchain.cmake 的 triple、processor、include 路径。
4. 若超出现有 vendor/stack/ABI/系统/GCC 组合，扩展 cross_build.py 的校验、共享镜像的工具链配置及对应测试，不能只改元数据绕过限制。
5. 增补 CI 矩阵及平台表，执行后续离线校验和真实 C/C++ smoke，再在目标上验收。

当前不固定 GCC 补丁版本，也不硬编码板端 glibc/CUDA/cuDNN 版本。镜像 apt 包会随发行版更新；正式发布需使用固定摘要镜像和包源快照，并保存 compiler --version、sysroot 清单及构建日志。
