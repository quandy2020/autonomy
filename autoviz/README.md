# Autoviz

Autonomy 原生 3D 机器人可视化工具，**零 ROS 依赖**，直接接入 **Autolink** 通信层，支持 Linux / Windows / macOS 独立部署。

> 架构：[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) · 部署：[docs/DEPLOYMENT.md](docs/DEPLOYMENT.md)

## 定位

| 维度 | Autoviz | Foxglove Bridge | RViz2 |
|------|------|-----------------|-------|
| 形态 | 原生桌面（Qt + OpenGL） | WebSocket 转发 | ROS 2 桌面 |
| 通信 | Autolink 直连 | Autolink → WebSocket | rclcpp |
| 部署 | 单可执行文件 + lib | 服务进程 | 完整 ROS 2 栈 |

## 目录结构

与 `autonomy/autonomy/` 相同：**单一工程**，源码按域分子目录。

```text
autoviz/
├── CMakeLists.txt          # 单一 target：autoviz 可执行文件
├── README.md
├── docs/
└── autoviz/                   # 源码根（对标 autonomy/autonomy/）
    ├── autoviz_main.cpp
    ├── rendering/          # 视口、网格、场景叠加
    ├── integration/        # Autolink 生命周期、Channel 发现
    ├── common/             # VisualizationManager
    ├── display/            # Display 插件（TF / LaserScan / Marker）
    ├── transform/          # 内置 TF Buffer（tf2）
    └── ui/                   # 主窗口
```

## 构建

依赖：`qt6-base-dev`、`libqt6svg6-dev`、OpenGL 3.3+、**automsgs**、**autolink**。**不链接** ROS / rviz / `libautonomy`。

```bash
cmake -B build -DBUILD_AUTOVIZ=ON
cmake --build build --target autoviz
./build/bin/autoviz
```

## 验证（配合 fakedata）

```bash
# 终端 1
./bin/autonomy_foxglove_fakedata

# 终端 2（可选加载 config/default.autoviz）
./build/bin/autoviz --config config/default.autoviz
```

默认 Display：TF、Scan、Marker、Path、Map、Odometry、PointCloud2。

**Displays 面板**：
- Fixed Frame / Show Grid 开关
- Channel 列双击下拉选择（从 Autolink 拓扑填充，可手动输入）
- File → Save Config 保存 `.autoviz`

## 相关文档

- [架构设计](docs/ARCHITECTURE.md)
- [跨平台部署](docs/DEPLOYMENT.md)
- [Foxglove Bridge](../autonomy/visualization/README.md)
