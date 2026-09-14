# 7. 运行验证

### 7.1 多进程栈验证

```bash
export PATH="$PWD/build/bin:$PATH"
export AUTOLINK_LAUNCH_PATH="$PWD/autonomy/system/launch"
export AUTONOMY_BT_PLUGIN_PATH="$PWD/build/lib"
export GLOG_logtostderr=1
autolink_launch autonomy.launch
```

成功标志：各进程（planning / control / task / …）启动无致命错误；发令后经 Bridge / Action 观察任务推进。`autonomy_nav_test` **已移除**。

### 7.2 检查编译产物

```bash
ls -la build/lib/libautonomy.so
ls -la build/bin/autonomy.planning build/bin/autonomy.control build/bin/autonomy.task
ls build/bin/autolink_launch
ls build/lib/*bt* 2>/dev/null | head
```

### 7.3 ROS 2 运行时验证

```bash
# 节点列表
ros2 node list

# 话题列表
ros2 topic list
ros2 topic echo /plan
ros2 topic info /cmd_vel

# 单节点信息
ros2 node info /<node_name>
```

### 7.4 可视化

```bash
# RViz2
rviz2

# Foxglove（autonomy.launch 可含 foxglove_bridge）
# 浏览器 / Studio 连接 ws://localhost:8765
```

地图、路径、TF 显示配置见 [13 Visualization](../13_Visualization/07_rviz2_ros2.md)。

### 7.5 Bridge / gRPC（若启用）

确认 Bridge 服务监听（端口见 `bridge.pb.txt`）。详见 [15 Bridge](../15_Bridge/00_guide.md)。

### 7.6 健康检查清单

| 检查项 | 命令 / 方法 |
|--------|-------------|
| 配置加载 | 启动日志无 conf 路径错误 |
| BT 插件 | `echo $AUTONOMY_BT_PLUGIN_PATH` 且目录含 `.so` |
| 地图加载 | planning 日志含 map / costmap 初始化 |
| TF | 仿真 / Bridge 发布 `map→base_link` |
| 发令 | Bridge / Action 有 Goal / Feedback |
| 控制 | Follow 阶段有 `cmd_vel` |

### 7.7 相关文档

- [§3 多进程栈](03_autonomy_process.md)
- [§7 故障排查](08_troubleshooting.md)
