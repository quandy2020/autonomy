# 6. Foxglove

你想用浏览器看 ROS 话题（地图、路径、激光），且栈已通过 autonomy_ros 暴露标准 topic。

[Foxglove Studio](https://foxglove.dev/) 连接 **foxglove_bridge**（WebSocket，默认 8765），不依赖域内 VisualizationServer。

![Foxglove Studio](./images/foxglove.png)

- **`VisualizationServer` 不存在**；`config/visualization/visualization.lua` **未被**主配置加载，改 lua 不会让 bridge 多订话题。  
- 纯 Autolink 栈、未起 autonomy_ros 时，需 **Autoviz** 或自建 bridge，Foxglove 不会自动看到 commsgs。  
- `install_mcap.sh` 装的是依赖；C++ 栈内**没有** MCAP Writer 产品接口。

| 情况 | 建议 |
|------|------|
| 已有 ROS 2 launch | Foxglove + bridge |
| 仅 autolink_launch | 先 Autoviz 或加 ROS 桥 |
| 远程运维 | 映射 8765 + TLS/SSH 隧道自管 |

### 接入链

```text
libautonomy → autonomy_ros（外部）→ ROS 2 topics → foxglove_bridge → ws://host:8765 → Studio
```

```bash
sudo apt install ros-humble-foxglove-bridge
ros2 launch autonomy_ros autonomy.launch.py   # 或你的 launch
ros2 run foxglove_bridge foxglove_bridge
# Studio: Open connection → Foxglove WebSocket → ws://localhost:8765
```

Docker 里常映射 8765（见 Running / Docker 文档）。离线：拖入 MCAP 或 bag 文件。

### 推荐面板

| 面板 | 订阅 | 用途 |
|------|------|------|
| 3D | `/map`, `/plan`, `/scan`, TF | 导航全貌 |
| Plot | `/cmd_vel`, odom | 控制曲线 |
| Image | 相机 topic | 视觉 |
| Raw Messages | 任意 | 排错 |

![实时曲线示例](./images/curve2.png)

### 常见坑

| 现象 | 先查 |
|------|------|
| Connection refused | bridge 进程、端口映射 |
| 有连接无 TF | `/tf`、`use_sim_time` |
| 与 gRPC Bridge 混淆 | autonomy/bridge 不发激光给 Foxglove |

### 延伸阅读

[§4 配置](04_configuration.md) · [§7 RViz2](07_rviz2_ros2.md)
