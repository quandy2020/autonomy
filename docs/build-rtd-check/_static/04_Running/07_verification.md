# 7. 运行验证

### 7.1 离线测试验证

`autonomy_nav_test` 成功标志：

```
Navigation succeeded.
Last path poses: <N>   # N >= 2
```

失败时检查 glog 输出中的 `GetPlan failed`、`Autonomy not ready`、`Transform` 等关键字。

### 7.2 检查编译产物

```bash
ls -la build/lib/libautonomy.so
ls -la build/bin/autonomy_nav_test
ls build/lib/autonomy_behavior_tree_*.so | head
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

# rqt
rqt
```

地图、路径、TF 显示配置见 [13 Visualization](../13_Visualization/07_rviz2_ros2.md)。

### 7.5 Bridge / gRPC（若启用）

```bash
# 确认 Bridge 服务监听（端口见 bridge 配置）
curl -v localhost:8765   # 示例，以实际配置为准
```

详见 [15 Bridge](../15_Bridge/00_guide.md)。

### 7.6 健康检查清单

| 检查项 | 命令 / 方法 |
|--------|-------------|
| 配置加载 | 启动日志无 `Configuration ... empty` |
| BT 插件 | `echo $AUTONOMY_BT_PLUGIN_PATH` 且目录含 `.so` |
| 地图加载 | 日志含 map / costmap 初始化 |
| TF | nav_test 内自动发布 `map→base_link` |
| 规划 | `Last path poses >= 2` |
| 控制 | 直驱/BT Follow 阶段有 `cmd_vel` 积分 |

### 7.7 相关文档

- [§4 离线导航测试](04_nav_test.md)
- [§8 故障排查](08_troubleshooting.md)
