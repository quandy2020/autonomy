# 7. 导航与 BT

### Q: `GetPlan failed` 或路径为空？

- 检查起终点是否在障碍物上
- 增大起终点距离
- 确认地图已加载、`MapServer::Start()` 成功

### Q: `planned path too short`？

起终点重合或不可达，调整坐标。

### Q: 导航超时？

增大 `--timeout_sec`；检查 controller 配置与里程计是否正常注入。

### Q: BT 插件 load 失败？

```bash
export AUTONOMY_BT_PLUGIN_PATH=build/lib
ls build/lib/*bt*  # 确认 .so 存在
```

插件源码目录 `autonomy/navigator/behavior_tree/plugins/` 当前可能为空，需等待迁回。

### Q: `--use_bt=true` 但仍走直驱？

`Autonomy::Configure()` 当前可能强制 `use_bt_navigation_ = false`。以源码为准，见 [17 Tasks · 执行模式](../17_Tasks/06_execution_modes.md)。

### Q: `TransformAvailable` 失败 / `no robot pose`？

- 确认 `global_frame` / `robot_base_frame` 与配置一致
- `autonomy_nav_test` 会自动注入 odom 与 TF；自定义程序需自行发布

### Q: BT 与 Nav2 行为不一致？

对比 `config/navigator/behavior_tree/*.xml` 与 nav2 对应 XML；检查 `navigator.lua` 中插件列表。

### Q: 任务 API 与 Navigator 文档看哪个？

- **任务语义 / API**： [17 Tasks](../17_Tasks/index.rst)
- **BT 实现 / 插件**： [16 Navigator](../16_Navigator/index.rst)
