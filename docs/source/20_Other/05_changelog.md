# 5. 版本记录

主要变更摘要来自仓库根目录 `CHANGELOG.rst`。

### 5.1 0.1.1（2026-05-19）

**架构**

- `autonomy/` 用 C++17 标准库替代 Boost
- 移除 `libautonomy` 对 Autolink 中间件的硬依赖（CMake 独立构建路径）
- `tasks` 源码并入 `libautonomy`，`TaskOptions` → `NavigatorOptions`

**Tasks / 行为树**

- 进程内 `BtActionServer` 替代 ROS/Autolink SimpleActionServer
- BT 插件去除 Autolink Node 依赖，改用黑板传参
- `BtNavigator` 仅依赖 `TaskOptions` / `NavigatorOptions`

**Control / Planning**

- `ControllerServer` / `PlannerServer` 可无 Autolink Action Server 运行
- 直驱规划接线

**构建**

- 移除 Bazel 构建文件
- 默认 `find_package(behaviortree_cpp REQUIRED)`

### 5.2 完整记录

```bash
# 仓库根目录
cat CHANGELOG.rst
```

在线查看：[CHANGELOG.rst](https://github.com/quandy2020/autonomy/blob/main/CHANGELOG.rst)

### 5.3 路线图

见 [01 Instructions · 版本与路线](../01_Instructions/08_roadmap.md)。
