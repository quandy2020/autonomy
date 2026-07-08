# 5. 工程结构

### 5.1 仓库目录

```text
autonomy/
├── autolink/              # 通信运行时（独立子项目）
├── autonomy/              # C++ 核心库源码
│   ├── bridge/            # gRPC 桥接
│   ├── commsgs/           # 消息定义（C++ + proto）
│   ├── common/            # 数学、日志、配置工具
│   ├── control/           # 局部控制器
│   ├── localization/      # 定位（Atlas VSLAM）
│   ├── map/               # 地图 / costmap / grid_map
│   ├── navigator/         # 行为树导航编排
│   ├── perception/        # 感知（扩展）
│   ├── planning/          # 全局规划
│   ├── prediction/        # 预测（扩展）
│   ├── sensor/            # 传感器同步
│   ├── system/            # Autonomy 顶层入口
│   ├── transform/         # TF
│   ├── tools/             # 工具库
│   └── vehicle/           # 车辆模型
├── config/                # Lua / YAML 配置
│   ├── autonomy.lua       # 顶层配置入口
│   ├── common.lua         # 跨模块共享参数
│   ├── planner/
│   ├── controller/
│   ├── navigator/
│   └── localization/
├── cmake/                 # CMake 模块
├── docker/                # Docker 镜像与脚本
├── docs/                  # Sphinx 文档（本手册）
├── scripts/               # install_deps 等
├── images/                # README 用图
├── CMakeLists.txt
└── version.json           # 版本号（当前 0.1.1）
```

### 5.2 构建产物

| 产物 | 路径 | 说明 |
|------|------|------|
| 核心库 | `build/lib/libautonomy.so` | 链接所有已启用模块 |
| 可执行文件 | `build/bin/` | 测试与工具 |
| BT 插件 | `build/lib/autonomy_behavior_tree_*.so` | 行为树节点 |
| 规划器插件 | `build/lib/*_planner*.so` | GlobalPlanner 插件 |

### 5.3 CMake 选项

| 选项 | 默认 | 说明 |
|------|------|------|
| `BUILD_GRPC` | ON | gRPC Bridge |
| `BUILD_TEST` | ON | 单元测试 |
| `BUILD_DOCS` | ON | Sphinx 文档 |
| `BUILD_PROMETHEUS` | OFF | Prometheus 指标 |
| `BUILD_ONNXRUNTIME` | ON | ONNX 推理（若找到运行时） |

### 5.4 配置系统

**Lua → Protobuf** 管线：

```
config/xxx/xxx.lua
      │
      ▼
LuaParameterDictionary
      │
      ▼
xxx::LoadOptions() → proto::XxxOptions
      │
      ▼
XxxServer(options)
```

**共享参数**（`config/common.lua`）：

| 参数 | 用途 |
|------|------|
| `global_frame` | 全局坐标系（默认 `map`） |
| `robot_base_frame` | 基座帧（默认 `base_link`） |
| `default_planner_id` | 默认规划器 |
| `default_controller_id` | 默认控制器 |
| `goal_reached_tolerance` | 目标容差 |

须在 `planner`、`controller`、`navigator` 三处保持一致。

### 5.5 依赖安装

```bash
# 完整安装（APT + 第三方脚本）
python3 -m install_deps

# 仅 APT
python3 -m install_deps --apt-only

# 列出 APT 包名
python3 -m install_deps --list-apt
```

主要第三方库：Eigen3、Ceres、glog、OpenCV、OSQP、yaml-cpp、Lua 5.3、Protobuf、BehaviorTree.CPP 4.x。

### 5.6 代码风格与格式化

```bash
python3 scripts/format.py autonomy/planning
python3 scripts/format.py autonomy/navigator
```

### 5.7 相关文档

- [§2 快速上手](02_quickstart.md)
- [02 Installation](../02_Installation/00_guide.md)
