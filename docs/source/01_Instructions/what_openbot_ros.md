# Autonomy简介

![Autonomy 框架架构](./images/openbot_framework.png)

## 1 概述

**autonomy_ros** 是基于 **autonomy** 核心框架构建的 ROS2 兼容层，旨在为 Autonomy 框架提供 ROS2 可视化调试和生态系统集成能力。

---

## 2 Autonomy核心框架

### 2.1 Autonomy_ROS核心框架

**Autonomy_ROS** 是 Autonomy 项目中的一个重要组件，它充当了 Autonomy 核心框架与 ROS2 生态系统之间的桥梁。通过 Autonomy_ROS，开发者可以：

- 使用 ROS2 工具进行可视化调试（如 RViz2、Foxglove Studio）
- 与现有的 ROS2 生态系统无缝集成
- 利用 ROS2 丰富的工具链和库
- 在保持 Autonomy 高性能的同时，享受 ROS2 的便利性

- **不是替代品**: Autonomy_ROS 不是用来替代 ROS2，而是作为 Autonomy 与 ROS2 之间的接口层
- **可选组件**: Autonomy 核心框架可以独立运行，不依赖 ROS2
- **双向兼容**: 支持 Autonomy 消息格式与 ROS2 消息格式的相互转换

> **Autonomy_ROS** 作为 Autonomy 的可选扩展，提供：
>
> - **ROS2 集成**: 使 Autonomy 组件可以作为 ROS2 节点运行
> - **工具支持**: 可以使用 ROS2 生态系统的各种工具进行调试和监控
> - **数据兼容**: 支持与 ROS2 项目的数据交换和互操作

### 2.2 Autonomy 核心框架

**Autonomy** 是一个独立的机器人系统框架，具有以下特点：

- **独立运行**: 不依赖 ROS/ROS2，可以完全独立运行
- **高性能**: 基于 Autolink 通信框架，提供低延迟、高吞吐量的通信能力
- **模块化**: 采用插件架构，支持动态加载和管理
- **跨平台**: 支持 x86-64 和 ARM64 架构

---

## 3 快速开始

### 3.1 编译 Autonomy_ROS

确保您已经编译了 Autonomy 核心框架，然后编译 Autonomy_ROS：

```bash
# 进入 Docker 容器
docker exec -it SpaceHero /bin/bash

# 设置 ROS2 环境
source /opt/ros/humble/setup.bash

# 编译项目（Autonomy_ROS 会自动编译）
cd /workspace/autonomy
colcon build --symlink-install
```

### 3.2 运行示例

```bash
# 设置环境
source /workspace/autonomy/install/setup.bash

# 启动 Autonomy_ROS 节点
ros2 run autonomy_ros <node_name>
```

### 3.3 可视化

在另一个终端中启动 RViz2：

```bash
source /opt/ros/humble/setup.bash
rviz2
```

---

## 4 详细文档

完整的 Autonomy_ROS 文档位于编译安装目录下的 `docs` 文件夹中。

![文档结构](./images/openbot_docs.png)

### 4.1 文档内容

详细文档包含以下内容：

- **API 参考**: 完整的 API 接口说明
- **消息类型**: 支持的消息类型和转换规则
- **配置指南**: 如何配置和使用 Autonomy_ROS
- **示例代码**: 实际使用示例
- **最佳实践**: 推荐的开发模式和注意事项

### 4.2 访问文档

编译完成后，文档位于：

```bash
# 在 Docker 容器中
cd /workspace/autonomy/install/autonomy/share/autonomy/docs

# 或在宿主机上
cd <autonomy_path>/install/autonomy/share/autonomy/docs
```
