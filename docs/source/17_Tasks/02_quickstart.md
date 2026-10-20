# 2. 快速开始

### 2.1 多进程发令

```bash
export PATH="$PWD/build/bin:$PATH"
export AUTOLINK_LAUNCH_PATH="$PWD/autonomy/system/launch"
export AUTONOMY_BT_PLUGIN_PATH="$PWD/build/lib"
autolink_launch autonomy.launch
# Bridge 或 Action Client → NavigateToPose
```

### 2.2 配置前提

| 项 | 路径 |
|----|------|
| Navigator | `config/navigator/navigator.lua` / `task/conf/navigator.pb.txt` |
| BT XML | `behavior_tree/*.xml` |
| 共享帧 | `common.lua` |
