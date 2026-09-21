# 2. 快速上手

完整依赖/板端步骤见 [Installation · 快速安装](../02_Installation/02_quickstart.md)。按域裁剪见 [Installation §6](../02_Installation/06_build.md)。

### 1. 克隆

```bash
git clone --recurse-submodules https://github.com/quandy2020/autonomy.git
cd autonomy && git submodule update --init --recursive
# monorepo：进入含 CMakeLists.txt 的源码根
```

要求：Ubuntu 22.04，CMake ≥ 3.20，GCC 11+，Ninja，内存 ≥ 8 GB。

### 2. 依赖与编译

```bash
python3 scripts/install_dependencies.py --skip-installed
mkdir -p build && cd build
cmake -G Ninja .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/usr/local
ninja -j$(nproc) && cd ..
ls build/lib/libautonomy_common.so build/lib/libautolink.so
```

### 3. 环境 + 启动

```bash
source scripts/setup_environment.bash   # 每个新终端

autolink_launch autonomy.launch         # 全栈联调
# 或仅导航三件套：autolink launch start task.launch
# 停止：           autolink launch stop task.launch
```

勿同时开 `autonomy.launch` 与 `task.launch`。感知/驱动等见对应 launch 与 [04 Running](../04_Running/02_quickstart.md)。

| 下一步 | 文档 |
|--------|------|
| 架构 / 能力 | [§3](03_system_architecture.md) |
| 仓库目录 / 生态 | [§4](04_repository.md) |
| 运行 / 通信 / 排错 | [Running](../04_Running/00_guide.md) · [Communication](../03_Communication/00_guide.md) · [排错](../02_Installation/08_troubleshooting.md) |
