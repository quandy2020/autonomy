# 9. Autoviz

不想装 ROS，但要桌面端看 Autolink 话题（地图、路径等）。

顶层包 `autoviz/`，`-DBUILD_AUTOVIZ=ON`。Qt + Ogre，直接订阅 Autolink；完整文档在 `autoviz/docs/`。

与 Foxglove（浏览器/WS）、RViz2（ROS 2）并列，三选一即可。

| 条件 | 工具 |
|------|------|
| 纯 Autolink 栈 | **Autoviz** |
| 只有 ROS 话题 | RViz2 / Foxglove |
| 远程 | Foxglove |

### 构建与运行

```bash
colcon build --packages-select autoviz --cmake-args -DAUTOLINK_BUILD_PYTHON=ON
source install/setup.bash && autoviz
```

或在编 autonomy 时 `-DBUILD_AUTOVIZ=ON`。Linux 需 Qt6 开发包；macOS 见 `autoviz/deploy/macos/README.md`。

### 常见坑

| 现象 | 说明 |
|------|------|
| 编不过 | Qt6/Ogre 依赖 |
| 与 VisualizationServer 混淆 | Server 未实现 |

### 延伸阅读

[00 指南](00_guide.md)
