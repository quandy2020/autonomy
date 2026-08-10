# Autoviz

Autonomy 原生 3D 机器人可视化工具，**零 ROS 依赖**，直接接入 **Autolink** 通信层，**跨平台**：Linux / **macOS** / Windows 独立部署。

> 架构：[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) · 部署：[docs/DEPLOYMENT.md](docs/DEPLOYMENT.md) · macOS：[deploy/macos/README.md](deploy/macos/README.md)

## 定位

| 维度 | Autoviz | Foxglove Bridge | RViz2 |
|------|------|-----------------|-------|
| 形态 | 原生桌面（Qt + OpenGL） | WebSocket 转发 | ROS 2 桌面 |
| 通信 | Autolink 直连 | Autolink → WebSocket | rclcpp |
| 部署 | 单可执行文件 + lib | 服务进程 | 完整 ROS 2 栈 |
| 平台 | Linux · macOS · Windows | 浏览器/服务 | 主要为 Linux |

## 目录结构

独立 CMake 工程，参考 QGC 的分层思路，保持 **7 个 cmake 模块 + 1 个根 CMakeLists**：

```text
autoviz/
├── CMakeLists.txt       # 引导：模式检测 → include 模块 → 摘要
├── CMakePresets.json    # debug / release 预设
├── cmake/
│   ├── Config.cmake     # 选项、工具链、Qt AUTOMOC
│   ├── Dependencies.cmake
│   ├── Sources.cmake
│   ├── App.cmake        # autoviz + bicmap 示例工具
│   ├── OgreBackend.cmake
│   ├── Tests.cmake      # 可选，-DBUILD_AUTOVIZ_TESTS=ON
│   └── Install.cmake
├── autoviz/             # C++ 源码
├── qml/                 # Qt Quick 3D 车辆预览
└── resources/
```

## 构建

依赖：Qt 6、OpenGL 3.3+、**automsgs**、**autolink**、yaml-cpp。**不链接** ROS / rviz / `libautonomy`。

### colcon（推荐）

在 workspace 根目录（含 `src/autonomy/autoviz/package.xml`）：

```bash
colcon build --packages-select autoviz --cmake-args -DAUTOLINK_BUILD_PYTHON=ON
source install/setup.bash
autoviz   # 或 ./install/autoviz/bin/autoviz
```

仅编译 `autonomy` 主包时默认**不再**内嵌 autoviz；若仍需超项目内嵌：

```bash
colcon build --packages-select autonomy --cmake-args -DBUILD_AUTOVIZ=ON
```

| 平台 | 依赖安装 | 构建要点 |
|------|----------|----------|
| **Linux** | `qt6-base-dev` `libqt6svg6-dev` … | `cmake -B build && cmake --build build --target autoviz` |
| **macOS** | `brew install qt@6 cmake ninja …` | `-DCMAKE_PREFIX_PATH="$(brew --prefix qt@6)"`，详见 [deploy/macos](deploy/macos/README.md) |
| **Windows** | MSVC + Qt6 安装器 | `-DCMAKE_PREFIX_PATH=C:\Qt\6.x\msvc2019_64`，见 [deploy/windows](deploy/windows/README.md) |

```bash
cd src/autonomy/autoviz   # 或本仓库 autoviz/
cmake -B build
cmake --build build --target autoviz
./build/bin/autoviz
```

macOS 示例：

```bash
cmake -B build -DCMAKE_PREFIX_PATH="$(brew --prefix qt@6)"
cmake --build build --target autoviz
./build/bin/autoviz
```

或使用预设 / 工具脚本：

```bash
cmake --preset release && cmake --build build --target autoviz
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

- [架构设计](docs/ARCHITECTURE.md)
- [跨平台部署](docs/DEPLOYMENT.md)
- [Foxglove Bridge](../autonomy/visualization/README.md)
