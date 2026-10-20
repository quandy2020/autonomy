# 4. 运行问题

### Q: 找不到 libautonomy.so？

```bash
export LD_LIBRARY_PATH=build/lib:$LD_LIBRARY_PATH
```

### Q: autonomy_nav_test？

已移除。改用：

```bash
export PATH="$PWD/build/bin:$PATH"
export AUTOLINK_LAUNCH_PATH="$PWD/autonomy/system/launch"
autolink_launch autonomy.launch
```

### Q: 配置加载失败？

核对 `autonomy.pb.txt` 与各模块 `--conf=`。

### Q: BT 未就绪？

```bash
export AUTONOMY_BT_PLUGIN_PATH=build/lib
```

### Q: TF 帧名错？

统一 `global_frame` / `robot_base_frame`。

### Q: ROS 2 节点起不来？

```bash
source /opt/ros/humble/setup.bash && ros2 node list
```
