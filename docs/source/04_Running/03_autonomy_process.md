# 3. Autonomy 进程

`system::Autonomy` 是顶层运行时，负责构造 Map / Planner / Controller / TF 等子系统并完成生命周期管理。

### 3.1 生命周期

```
CreateAutonomy(options)
    → Start()        # 启动 MapServer、PlannerServer、ControllerServer…
    → Configure(runtime)   # 加载 navigator 配置、附着 BT（若启用）
    → [运行循环]     # NavigateToPose / 外部发令
    → Shutdown()     # 释放资源
```

### 3.2 主程序入口

源码：`autonomy/system/main.cpp`

```bash
# 典型 gflags（见 autonomy/common/gflags.hpp）
./build/bin/<autonomy_main> \
  --configuration_directory=config \
  --configuration_basename=autonomy.lua
```

主程序行为：

1. `autolink::Init`
2. `CreateOptions` 加载 `autonomy.lua`
3. `CreateAutonomy` → `Start()` → `Configure(runtime)`
4. 信号处理 `SIGINT`/`SIGTERM` 后 `Shutdown`

> **注意**：主程序默认**不自动下发导航目标**，仅保持进程运行。发令测试请用 [§4 nav_test](04_nav_test.md) 或上层 Bridge / Action Client。

### 3.3 RuntimeOptions

| 字段 | 说明 | 默认 |
|------|------|------|
| `use_bt_navigation` | 是否 BT 导航 | `true`（main）；nav_test 可覆盖 |
| `enable_bt_tasks` | 启用 BT 任务引擎 | `true` |
| `config_directory` | 配置根目录 | gflags |
| `planner_id` / `controller_id` | 覆盖默认插件 | 来自 `navigator.lua` |
| `global_frame` / `robot_base_frame` | 坐标系 | `map` / `base_link` |
| `goal_tolerance` | 目标容差 | `navigator.lua` |

### 3.4 对外 API（节选）

```cpp
autonomy->Start();
autonomy->Configure(runtime);

bool ok = autonomy->NavigateToPose(goal, cancel_checker, keep_alive, timeout_sec);

autonomy->ReplanToGoal(goal);
autonomy->Shutdown();
```

### 3.5 日志

```bash
export GLOG_logtostderr=1
export GLOG_minloglevel=0
```

详见 [02 Installation · 环境配置](../02_Installation/07_environment.md)。

### 3.6 相关文档

- [§4 离线导航测试](04_nav_test.md)
- [16 Navigator](../16_Navigator/00_guide.md)
