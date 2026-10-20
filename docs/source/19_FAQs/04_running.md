# 4. 运行问题

常见症状：动态库找不到、launch 失败、BT 插件未加载、Bridge 无监听，或仍按已删除的 `CreateAutonomy` 操作。

主栈是 **多进程 + autolink_launch**；TaskServer 编排，BT 在 `autonomy/task/`。无 VisualizationServer、SimulationServer 属预期。Bridge 需 `BUILD_GRPC=ON` 且 `autonomy.bridge` 在跑。

## 问答

### Q: 找不到 libautonomy*.so？

```bash
source scripts/setup_environment.bash   # 若有
export LD_LIBRARY_PATH="$PWD/build/lib:$LD_LIBRARY_PATH"
```

安装前缀构建则用 `install/setup.bash`。

### Q: autonomy_nav_test / CreateAutonomy？

已删除。请：

```bash
export PATH="$PWD/build/bin:$PATH"
export AUTOLINK_LAUNCH_PATH="$PWD/autonomy/system/launch"
export AUTONOMY_BT_PLUGIN_PATH="$PWD/build/lib"
autolink_launch autonomy.launch
```

场外发令走 Bridge 或 Action Client。

### Q: 配置加载失败？

核对 launch 里各模块 `--conf=`；`AUTONOMY_PATH` 是否含 `autonomy/task/conf`。

### Q: BT 未就绪 / 插件错误？

```bash
export AUTONOMY_BT_PLUGIN_PATH="$PWD/build/lib"
```

确认 `autonomy.task` 在跑且 navigator pb 存在。

### Q: TF 帧名不一致？

全栈统一 `global_frame`（常 `map`）、`robot_base_frame`（常 `base_link`）。

### Q: Bridge 端口无监听？

查 `autonomy.bridge`、`BUILD_GRPC=ON`、`bridge.pb.txt` 的 port。

### Q: ROS 2 侧节点（混用时）？

```bash
source /opt/ros/humble/setup.bash && ros2 node list
```

Autonomy 主栈不依赖 ROS 2 跑导航。
