# 2. 快速运行

> 前提：已完成 [02 Installation](../02_Installation/02_quickstart.md) 编译。

### 2.1 启动多进程栈

```bash
source scripts/setup_environment.bash
# 等价于手动设置 PATH / AUTOLINK_LAUNCH_PATH / AUTONOMY_BT_PLUGIN_PATH / GLOG_*

autolink_launch autonomy.launch
```

将拉起 `monitor` / `planning` / `control` / `task` / `perception` / `bridge` / `foxglove` 等进程。发令请用 Bridge、autolink Action Client 或上层业务。

### 2.2 探索闭环（可选）

```bash
autolink_launch exploration.launch
```

与 `autonomy.launch` 进程有重叠，**不要同时启动**。

### 2.3 配置与地图

- 共享快照：`autonomy/system/conf/autonomy.pb.txt`（`--conf=autonomy.pb.txt`）
- 地图等资产：`autonomy/map/conf/`
- BT XML：`autonomy/task/conf/behavior_tree/`

### 2.4 下一步

| 目标 | 文档 |
|------|------|
| 进程与配置说明 | [§3 多进程栈](03_autonomy_process.md) |
| Docker 内运行 | [§4 Docker 运行时](05_docker_runtime.md) |
| 验证清单 | [§6 运行验证](07_verification.md) |
