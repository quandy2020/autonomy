# 4. 运行问题

### Q: `libautonomy.so: cannot open shared object file`？

```bash
export LD_LIBRARY_PATH=build/lib:$LD_LIBRARY_PATH
# 或从 build/bin 目录运行
```

### Q: 找不到 `autonomy_nav_test`？

目标可能未接入顶层 CMake。检查 `autonomy/system/tools/` 是否被 `add_subdirectory`，见 [18 Tools · nav_test](../18_Tools/04_nav_test.md#44-构建说明)。

### Q: `Configuration directory empty` / 配置加载失败？

```bash
./build/bin/autonomy_nav_test --configuration_directory=config
```

确认 `config/autonomy.lua` 存在。

### Q: `Autonomy not ready`？

检查 BT 配置与插件路径：

```bash
export AUTONOMY_BT_PLUGIN_PATH=build/lib
```

### Q: TF / 帧名错误？

统一 `config/common.lua` 中的 `global_frame` 与 `robot_base_frame`，与各子模块配置保持一致。

### Q: ROS 2 节点无法启动？

```bash
source /opt/ros/humble/setup.bash
ros2 node list
```

详见 [04 Running · ROS 2](../04_Running/06_ros2_integration.md)。
