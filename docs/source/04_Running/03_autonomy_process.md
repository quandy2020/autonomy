# 3. 多进程栈

## 背景

各域独立进程 + Autolink IPC。没有进程内 `CreateAutonomy` 一类聚合入口。

## 思路

Autonomy 是「一域一进程 + Autolink IPC」，没有单体聚合入口。排障按 launch 清单核对缺哪条 Server；最小闭环可只起 `task.launch`（planning + control + task）。

## 步骤

### 1. 入口

源码：`autonomy/system/launch/autonomy.launch`

```bash
source scripts/setup_environment.bash
autolink_launch autonomy.launch
```

只要导航三件套：`autolink launch start task.launch`（见 `autonomy/task/launch/task.launch`）。

### 2. 进程

| 二进制 | 配置 | 职责 |
|--------|------|------|
| `autonomy.monitor` | `--conf=monitor.pb.txt` | 健康监控 |
| `autonomy.planning` | `--conf=autonomy.pb.txt` | PlannerServer |
| `autonomy.control` | `--conf=autonomy.pb.txt` | ControllerServer |
| `autonomy.task` | gflags（BT 根目录等） | TaskServer / BT |
| `autonomy.perception` | `--conf=autonomy.pb.txt` | PerceptionServer |
| `autonomy.bridge` | `--conf=bridge.pb.txt` | 外部桥接 |
| `autonomy.foxglove_bridge` | host/port gflags | 可视化 |

各 `*_main.cpp`：`CreateOptions`（或本地 conf）→ Server → `autolink::WaitForShutdown`。

### 3. 共享配置

`system::CreateOptions("autonomy.pb.txt")` 加载 `AutonomyOptions`。planning / control / perception 只取子字段。模块本地参数：`autonomy/<mod>/conf/`。

### 4. 发令

主栈**不**自动下发导航目标。用 Bridge、Action / Service Client 或上层业务。

### 5. 日志

```bash
export GLOG_logtostderr=1
export GLOG_minloglevel=0
```
