# 7. 导航与 BT

常见症状：GetPlan 失败、path 过短、Navigate 一直 RUNNING、BT 插件 load 失败、Bridge OK 但车不动。

**TaskServer** 收令并跑 **BT**（代码在 `autonomy/task/`）。先 map/TF/costmap，再 planning/control 日志；Bridge **BUSY** 与 Task `enable_*` 不一致时常见「有响应无运动」。

## 问答

### Q: GetPlan 失败或 path 为空？

先 `SetCurrentMap`、起终点在 free space、TF `map`→`base` 可用；看 Planning 第一条 FATAL。

### Q: planned path too short？

起终点同格或 goal 在 lethal 边缘；用 RViz/autoviz 点选验证（autoviz 需 `BUILD_AUTOVIZ=ON`）。

### Q: 导航一直 RUNNING 不到点？

- GoalChecker 容差与 BT `GoalReached` 不一致  
- Recovery 循环 → 查 costmap、里程计  
- Bridge：Navigate stream 的 `state`、`remaining_distance_meters`

### Q: 超时？

增大 navigator / Navigate `timeout_seconds`；查 Recovery 或 LocalSurvival。

### Q: BT 插件 load 失败？

```bash
export AUTONOMY_BT_PLUGIN_PATH="$PWD/build/lib"
ls build/lib/*behavior_tree*
```

名与 `navigator.pb.txt` 的 `plugin_lib_names` 一致（无 `lib` 前缀、无 `.so`）。

### Q: TransformAvailable 一直失败？

帧名与 common 一致；仿真是否发 TF。

### Q: Bridge 返回 OK 但车不动？

TaskServer 未起或槽位 BUSY；`GetActiveGoal` + task/planning/control 进程。

### Q: 看 Tasks 还是 Navigator？

任务与配置 → [17](../17_Tasks/00_guide.md)。BT 树与插件 → [16](../16_Navigator/00_guide.md)。

### Q: BT 还是直驱？

见 [17 · §6](../17_Tasks/06_execution_modes.md)。
