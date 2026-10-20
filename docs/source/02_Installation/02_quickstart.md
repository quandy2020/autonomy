# 2. 快速安装（本机 Ubuntu）

## 背景

面向 **Ubuntu 22.04 x86-64 / aarch64 宿主机**，需要从零到 `which autolink` 可用的逐步清单。Docker / 板端走其它章节。

## 思路

五步串行：确认源码根 → 装依赖 → Release 构建 → source 环境 → 按目标跳 Running / Docker / 板端。每一步在前一步成功后再继续。


## 步骤

### 1. 克隆源码

```bash
git clone --recurse-submodules https://github.com/quandy2020/autonomy.git
# 国内可选：https://gitee.com/quanduyong/autonomy.git
cd autonomy
git submodule update --init --recursive
```

确认你在 **源码根**（能看到顶层 `CMakeLists.txt`、`scripts/`、`docker/`）。

若仓库在更大 monorepo 里，进入含上述文件的子目录，例如：

```bash
cd /path/to/workspace/src/autonomy
```

### 2. 安装依赖

需 `sudo`。首次会编译 OpenCV / Ceres 等，耗时长、吃内存。

```bash
python3 scripts/install_dependencies.py --skip-installed
```

| 场景 | 命令 |
|------|------|
| 只要 APT，第三方已就绪 | `python3 scripts/install_dependencies.py --apt-only` |
| 中断后续装 | `python3 scripts/install_dependencies.py --resume-from install_opencv.sh --skip-installed` |
| 板端精简集 | 见 [§9](09_embedded_board.md)，用 `--profile board` |

默认 profile 为 `full`（桌面/CI，含 Assimp/Ogre 等）。细节与验证见 [§4](04_dependencies.md)。

### 3. 配置并编译

```bash
mkdir -p build && cd build
cmake -G Ninja .. \
-DCMAKE_BUILD_TYPE=Release \
-DCMAKE_PREFIX_PATH=/usr/local \
-DCMAKE_INSTALL_PREFIX=/usr/local
ninja -j$(nproc)
cd ..
```

未装 Ninja 时把 `-G Ninja` 去掉，并用 `cmake --build build -j$(nproc)`。

内存不足：`ninja -j2` 或 `ninja -j4`。

只要部分模块：见 [§6.2 模块化编译](06_build.md)。

### 4. 加载环境并验证

```bash
source scripts/setup_environment.bash

ls build/lib/libautonomy_common.so
ls build/lib/libautolink.so
which autolink
autolink --help | head
```

可选安装到系统前缀（新终端用 install 版 setup）：

```bash
sudo cmake --install build --prefix /usr/local
source /usr/local/share/autonomy/setup.bash
```

### 5. 下一步

| 目标 | 文档 |
|------|------|
| 每个新终端如何配环境 | [§7 环境配置](07_environment.md) |
| 启动导航 / task | [04 Running](../04_Running/00_guide.md) |
| Docker | [§5](05_docker.md) |
| 嵌入式板 | [§9](09_embedded_board.md) |
| 编译失败 | [§8](08_troubleshooting.md) |
