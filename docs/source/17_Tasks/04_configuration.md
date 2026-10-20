# 4. 任务配置

**什么时候改：** 换 BT、加插件、调帧名/容差/超，或 task 进程启动报 conf 错。

### 加载链

```text
common（帧、容差）→ navigator.pb.txt / task conf → NavigatorOptions → TaskServer::Configure
```

| 资产 | 典型路径 |
|------|----------|
| Navigator pb | `autonomy/task/conf/navigator.pb.txt` |
| BT XML | `autonomy/task/conf/behavior_tree/navigation/*.xml` |
| Task 进程 conf | `autonomy/task/conf/`（随 launch 的 `--conf=`） |
| Bridge（上游） | `autonomy/bridge/conf/bridge.pb.txt` |

Lua 若存在，多作为生成 pb 的源；**运行时以 pb + launch 为准**。

### 环境变量

| 变量 | 作用 |
|------|------|
| `AUTONOMY_BT_PLUGIN_PATH` | BT 插件 `.so` 搜索 |
| `AUTOLINK_LAUNCH_PATH` | launch 文件 |
| `AUTONOMY_PATH` | 模块 conf / behavior_tree 相对根（见 task_main 日志） |

### TaskServer 选项（概念）

`TaskServerOptions`（proto）：`scheduler`（互斥导航、反馈周期、默认超时）、`apps`（enable_navigation / teleop / …）。  
与 Bridge `CapabilitiesOptions` 应对齐，避免对外宣称支持但 task 未启用。

### 常见坑

| 现象 | 说明 |
|------|------|
| 改 lua 不生效 | 未重生成或未改 pb |
| 插件列表重复/缺项 | 对照 `build/lib/libautonomy_behavior_tree_*` |
| Bridge 能 Navigate task 无反应 | `enable_navigation` false |
