# Autoviz

Autonomy 原生 3D 机器人可视化工具，**零 ROS 依赖**，直接接入 **Autolink** 通信层，**跨平台**：Linux / **macOS** / Windows 独立部署。

> 文档索引：[docs/README.md](docs/README.md) · 架构：[docs/architecture/overview.md](docs/architecture/overview.md) · 部署：[docs/guide/deployment.md](docs/guide/deployment.md) · macOS：[deploy/macos/README.md](deploy/macos/README.md)

## 定位

| 维度 | Autoviz | Foxglove Bridge | RViz2 |
|------|------|-----------------|-------|
| 形态 | 原生桌面（Qt + OpenGL） | WebSocket 转发 | ROS 2 桌面 |
| 通信 | Autolink 直连 | Autolink → WebSocket | rclcpp |
| 部署 | 单可执行文件 + lib | 服务进程 | 完整 ROS 2 栈 |
| 平台 | Linux · macOS · Windows | 浏览器/服务 | 主要为 Linux |

## 目录结构

独立 CMake 工程，基于 [autocmake](../autocmake/)：

```text
autoviz/
├── CMakeLists.txt       # autocmake_project / binary / package
├── package.xml          # 依赖清单（autolink、automsgs、…）
├── cmake/
│   ├── Functions.cmake        # 源码收集 / Ogre / 桌面安装
│   └── apply_ogre_patches.sh
├── autoviz/             # C++ 源码
└── resources/
```

## 构建

依赖：Qt 6、**Ogre 1.x**（默认 auto-vendor 1.12.10）、**automsgs**、**autolink**、yaml-cpp。**不链接** ROS / rviz / `libautonomy`。视口为 Ogre，不使用纯 OpenGL 后端。

### colcon / autocmake 工作空间

在 workspace 根目录（含 `src/autonomy/autoviz/package.xml`）：

```bash
colcon build --packages-select autoviz --cmake-args -DAUTOLINK_BUILD_PYTHON=ON
source install/setup.bash
autoviz   # 或 ./install/autoviz/bin/autoviz
```

或：

```bash
../autocmake/scripts/autocmake build --base-path . --packages-select autoviz
```

仅编译 `autonomy` 主包时若仍需超项目内嵌：

```bash
colcon build --packages-select autonomy --cmake-args -DBUILD_AUTOVIZ=ON
```

| 平台 | 依赖安装 | 构建要点 |
|------|----------|----------|
| **Linux** | `qt6-base-dev` `libqt6svg6-dev` … | `cmake -B build && cmake --build build --target autoviz_app` |
| **macOS** | `brew install qt@6 cmake ninja …` | `-DCMAKE_PREFIX_PATH="$(brew --prefix qt@6)"`，详见 [deploy/macos](deploy/macos/README.md) |
| **Windows** | MSVC + Qt6 安装器 | `-DCMAKE_PREFIX_PATH=C:\Qt\6.x\msvc2019_64`，见 [deploy/windows](deploy/windows/README.md) |

```bash
cd src/autonomy/autoviz   # 或本仓库 autoviz/
cmake -B build
cmake --build build --target autoviz_app
./build/bin/autoviz          # 链接 build/lib/libautoviz.*
```

macOS 示例：

```bash
cmake -B build -DCMAKE_PREFIX_PATH="$(brew --prefix qt@6)"
cmake --build build --target autoviz_app
./build/bin/autoviz
```

或使用工具脚本：

```bash
python3 tools/configure.py && python3 tools/build.py
```

仍可作为 Autonomy 超项目子目录构建：`cmake -B build -DBUILD_AUTOVIZ=ON`（自 `src/autonomy` 目录）。

## 验证（配合 fakedata）

```bash
# 终端 1
./bin/autonomy_foxglove_fakedata

# 终端 2（可选加载 config/default.autoviz）
./build/bin/autoviz --config config/default.autoviz
```

## 相关文档

- [文档索引](docs/README.md)
- [架构总览](docs/architecture/overview.md)
- [构建与部署](docs/guide/build.md) · [docs/guide/deployment.md](docs/guide/deployment.md)
- [RViz2 对齐](docs/parity/README.md)
- [Foxglove Bridge](../autonomy/visualization/README.md)
