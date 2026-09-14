# 4. 运行问题

### Q: `libautonomy.so: cannot open shared object file`？

```bash
export LD_LIBRARY_PATH=build/lib:$LD_LIBRARY_PATH
# 或从 build/bin 目录运行
```

### Q: `autonomy_nav_test` 找不到？

该工具**已移除**。请改用多进程栈：

```bash
export PATH="$PWD/build/bin:$PATH"
export AUTOLINK_LAUNCH_PATH="$PWD/autonomy/system/launch"
autolink_launch autonomy.launch
```

见 [04 Running · 快速运行](../04_Running/02_quickstart.md)。

### Q: 配置加载失败？

确认 `autonomy/system/conf/autonomy.pb.txt` 与各模块 `--conf=` 路径正确；共享快照用 `CreateOptions`。

### Q: Task / BT 未就绪？

检查 BT 配置与插件路径：

```bash
export AUTONOMY_BT_PLUGIN_PATH=build/lib
```

### Q: TF / 帧名错误？

统一共享 conf / 模块 conf 中的 `global_frame` 与 `robot_base_frame`。

### Q: ROS 2 节点无法启动？

```bash
source /opt/ros/humble/setup.bash
ros2 node list
```

详见 [04 Running · ROS 2](../04_Running/06_ros2_integration.md)。
