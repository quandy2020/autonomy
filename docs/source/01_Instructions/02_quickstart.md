# 2. 快速上手

完整步骤见 [Installation · 快速安装](../02_Installation/02_quickstart.md)。按域裁剪见 [Installation §6](../02_Installation/06_build.md)。

### 1. 克隆

```bash
git clone --recurse-submodules https://github.com/quandy2020/autonomy.git
cd autonomy && git submodule update --init --recursive
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
source scripts/setup_environment.bash
autolink_launch autonomy.launch
# 仅导航：autolink launch start task.launch
# 停止： autolink launch stop task.launch
```

勿同时开 `autonomy.launch` 与 `task.launch`。
