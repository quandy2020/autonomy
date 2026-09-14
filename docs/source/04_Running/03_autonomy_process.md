# 3. 多进程栈

Autonomy 以 **独立进程 + autolink IPC** 运行，不再提供进程内 `CreateAutonomy` 聚合入口。

### 3.1 推荐入口

源码 launch：`autonomy/system/launch/autonomy.launch`

```bash
export PATH="$PWD/build/bin:$PATH"
export AUTOLINK_LAUNCH_PATH="$PWD/autonomy/system/launch"
autolink_launch autonomy.launch
```

### 3.2 进程一览

| 二进制 | 配置 | 职责 |
|--------|------|------|
| `autonomy.monitor` | `--conf=monitor.pb.txt` | 健康监控 |
| `autonomy.planning` | `--conf=autonomy.pb.txt` | PlannerServer |
| `autonomy.control` | `--conf=autonomy.pb.txt` | ControllerServer |
| `autonomy.task` | gflags（BT 根目录等） | TaskServer / BT |
| `autonomy.perception` | `--conf=autonomy.pb.txt` | PerceptionServer |
| `autonomy.bridge` | `--conf=bridge.pb.txt` | 外部桥接 |
| `autonomy.foxglove_bridge` | host/port gflags | 可视化 |

各 `*_main.cpp` 自行 `CreateOptions`（或模块本地 conf）→ 构造对应 Server → `autolink::WaitForShutdown`。

### 3.3 共享配置快照

`system::CreateOptions("autonomy.pb.txt")` 加载 [`AutonomyOptions`](../../autonomy/system/proto/autonomy_options.proto) 文本。planning / control / perception 只取各自子字段；**不是**进程内组装整栈。

模块本地参数见 `autonomy/<mod>/conf/`。

### 3.4 发令

主栈启动后**不会**自动下发导航目标。请通过：

- Bridge（gRPC / 外部接口）
- autolink Action / Service Client
- 上层业务进程

### 3.5 日志

```bash
export GLOG_logtostderr=1
export GLOG_minloglevel=0
```

详见 [02 Installation · 环境配置](../02_Installation/07_environment.md)。

### 3.6 相关文档

- [§2 快速运行](02_quickstart.md)
- [17 Tasks](../17_Tasks/00_guide.md)
- [15 Bridge](../15_Bridge/00_guide.md)
