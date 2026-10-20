# 4. 任务配置

```text
common.lua → navigator.lua → NavigatorOptions → TaskServer
```

| 文件 | 用途 |
|------|------|
| `config/navigator/navigator.lua` | 帧、容差、BT、插件 |
| `autonomy/task/conf/navigator.pb.txt` | 进程 conf |
| `behavior_tree/*.xml` | BT |
| `task_options.lua` | 兼容垫片（若存在） |

| 环境变量 | 作用 |
|----------|------|
| `AUTONOMY_BT_PLUGIN_PATH` | 插件 `.so` |
| `AUTOLINK_LAUNCH_PATH` | launch 搜索 |
