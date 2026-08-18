# Autoviz 跨平台部署

Autoviz 是 **Autolink 原生** 桌面可视化工具，**不链接、不运行 ROS/ros2/rclcpp/rviz**。可在 Linux / Windows / macOS 上独立部署。

## 运行时依赖

| 组件 | 用途 |
|------|------|
| **Qt 6** | Core / Gui / Widgets / OpenGL / Xml / Svg |
| **OpenGL 3.3+** | 默认渲染后端 |
| **automsgs** | Protobuf 消息定义 |
| **autolink** | Channel 通信、TF、`.record` 回放 |
| **yaml-cpp** | 配置读写 |
| **Ogre 1.x**（可选） | `-DAUTOVIZ_USE_OGRE=ON` 时启用 |

## 构建

```bash
cmake -B build -DBUILD_AUTOVIZ=ON
cmake --build build --target autoviz
```

Windows（MSVC + Qt6）：

```powershell
cmake -B build -DBUILD_AUTOVIZ=ON -DCMAKE_PREFIX_PATH=C:\Qt\6.x\msvc2019_64
cmake --build build --target autoviz --config Release
```

macOS（Intel / Apple Silicon，详见 [`deploy/macos/README.md`](../deploy/macos/README.md)）：

```bash
brew install cmake ninja qt@6 yaml-cpp protobuf

# 独立工程（在 autoviz/ 下）
cmake -S . -B build -DCMAKE_PREFIX_PATH="$(brew --prefix qt@6)"
cmake --build build --target autoviz

# 或超工程
cmake -B build -DBUILD_AUTOVIZ=ON -DCMAKE_PREFIX_PATH="$(brew --prefix qt@6)"
cmake --build build --target autoviz
```

可执行文件使用 `@loader_path` RPATH。若仍缺 `.dylib`：

```bash
export DYLD_LIBRARY_PATH="$PWD/build/lib:${DYLD_LIBRARY_PATH:-}"
./build/bin/autoviz
```

## 环境变量

| 变量 | 平台分隔符 | 说明 |
|------|-----------|------|
| `AUTOVIZ_PLUGIN_PATH` | `:` (Unix) / `;` (Windows) | Display / Tool / View / Transformer 插件目录 |
| `AUTOVIZ_RESOURCE_PATH` | 同上 | `package://` URDF mesh 搜索前缀（替代 ROS `AMENT_PREFIX_PATH`） |

示例（Linux）：

```bash
export AUTOVIZ_RESOURCE_PATH=/opt/autonomy/share:/home/user/robot_assets
export AUTOVIZ_PLUGIN_PATH=/opt/autonomy/lib/autoviz_plugins
./bin/autoviz --config share/autonomy/autoviz/default.autoviz
```

## 配置格式

| 格式 | 说明 |
|------|------|
| **`.autoviz`** | 原生 Autolink 会话配置（推荐） |
| **`.rviz`** | 只读导入兼容（Display/View/Panel 映射），**不依赖 ROS 运行时** |

## 回放与导入

- **原生格式**：Autolink `.record`（File → Open Record、拖入窗口或 `autoviz file.record` 后直接播放）
- **Legacy `.bag`**：可选外部工具 `bag_to_record`（Autolink 开发者工具；兼容旧名 `rosbag_to_record`）
- **`.mcap`**：待 autolink 原生 `mcap_to_record` 落地；当前脚本为占位

Autoviz 本体 **不要求** 安装 ROS 2 即可运行。

## 与 ROS 的关系

| 项 | 状态 |
|----|------|
| 编译链接 ROS | ❌ 无 |
| 运行时 rclcpp | ❌ 无 |
| RViz 插件 API | ❌ 不兼容（概念对齐，自研 `AUTOVIZ_PLUGIN_PATH`） |
| 消息类型字符串 | ✅ 兼容 `sensor_msgs/LaserScan` 等别名（映射到 automsgs） |
| `.rviz` 配置导入 | ✅ 离线 YAML 解析 |
| tf2 | ✅ 内嵌 vendored `autoviz/transform/tf2/` |
| `package://` mesh | ✅ 通过 `AUTOVIZ_RESOURCE_PATH`，不读 ament/colcon |

若需与 ROS 2 栈互通，请使用仓库中的 **`autonomy_ros` 桥接**（独立进程），Autoviz 仍只连 Autolink。

## 安装布局（推荐）

```text
prefix/
├── bin/autoviz
├── lib/libautolink.so ...
├── share/autonomy/autoviz/
│   ├── default.autoviz
│   └── scripts/mcap_to_record.py
└── lib/autoviz_plugins/          # 可选用户插件
```

设置 `RPATH`/`@loader_path` 后，可将 `bin/autoviz` 与 `lib/` 一并打包分发，无需系统级 ROS。
