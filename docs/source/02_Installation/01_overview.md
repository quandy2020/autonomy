(installation-overview)=
# 1. 安装概览

### 1.1 安装目标

完成安装后，您应获得：

| 产物 | 路径 | 说明 |
|------|------|------|
| 核心库 | `build/lib/libautonomy.so` | 链接所有已启用模块 |
| 可执行文件 | `build/bin/` | 测试与工具 |
| BT 插件 | `build/lib/autonomy_behavior_tree_*.so` | 行为树节点（若启用） |
| 文档（可选） | `docs/build/` | Sphinx HTML |

### 1.2 两种安装路径

| 路径 | 适用 | 步骤概要 |
|------|------|----------|
| **宿主机原生** | 日常开发、CI | `install_deps` → `cmake` + `ninja` |
| **Docker 容器** | 环境隔离、多架构 | `run_autonomy.py` → 容器内同上 |

两种方式均使用 **CMake + Ninja**，**不强制依赖 ROS 2**。

### 1.3 依赖层次

```
系统 (Ubuntu 22.04)
  ├── APT 包（cmake, eigen, protobuf, lua…）
  └── 第三方脚本 docker/install/*.sh
        ├── glog / gflags / Ceres / OpenCV
        ├── OSQP / BehaviorTree.CPP
        └── gRPC（BUILD_GRPC=ON 时）
              │
              ▼
        CMake 配置 + Ninja 编译
              │
              ▼
        libautonomy.so
```

### 1.4 与 ROS 2 的关系

| 方式 | 说明 |
|------|------|
| **推荐（当前）** | 纯 CMake 构建 `libautonomy`，无 ROS 2 运行时依赖 |
| **可选** | Docker 镜像内可预装 ROS 2 Humble，用于可视化或与 ROS 栈共存 |
| **不推荐作为主路径** | 旧版 `colcon build` 文档已过时，以本章 CMake 流程为准 |

ROS 2 集成运行见 [04 Running](../04_Running/00_guide.md)。

### 1.5 相关文档

- [§2 快速安装](02_quickstart.md)
- [01 Instructions · 快速上手](../01_Instructions/02_quickstart.md)
