# 2. 快速上手

## 背景

目标是从克隆仓库到能执行 `autolink_launch`，在一条最短路径上完成依赖、编译与环境加载。板端、Docker 与按域裁剪不在本页展开。

## 思路

顺序固定：子模块齐全 → `install_dependencies.py` → Ninja Release 构建 → `setup_environment.bash`。只要导航三件套时用 `task.launch`，且勿与全栈 `autonomy.launch` 同时开。

逐步说明与分支见 [Installation · 快速安装](../02_Installation/02_quickstart.md)；按域裁剪见 [Installation §6](../02_Installation/06_build.md)。


## 步骤

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
```

### 4. 仅 planning + control + task（冒烟）

例如 Autoviz 点目标：

```bash
autolink launch start task.launch
# 停止：autolink launch stop task.launch
```

勿同时开 `autonomy.launch` 与 `task.launch`（或 `exploration.launch`）。
