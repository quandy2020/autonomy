(nav-test-tool)=
# 4. 离线导航测试

**`autonomy_nav_test`** 在**不依赖 ROS** 的环境下验证完整导航栈：Map → Planner → Controller → Task/BT。

源码：`autonomy/system/tools/nav_test.cpp`

### 4.1 与其它工具对比

| 工具 | 路径 | 覆盖范围 | 状态 |
|------|------|----------|------|
| **`autonomy_nav_test`** | `system/tools/` | 完整栈 + 仿真 odom/TF | ✅ 有源码 |
| `autonomy_planning_test` | `planning/tools/` | 仅全局规划 | ❌ 目录不存在 |
| `autonomy_controller_test` | `control/tools/` | 规划 + 控制闭环 | ❌ 目录不存在 |
| Autonomy 主进程 | `system/main.cpp` | Start + Configure，不发目标 | ✅ |

### 4.2 功能流程

1. `CreateOptions` + `CreateAutonomy` 加载 `config/autonomy.lua`
2. `Start()` 启动各 Server
3. `Configure()` 附着 BT（演进中）
4. 注入初始里程计与 TF
5. 后台线程按 `cmd_vel` 积分仿真位姿
6. 主线程调用 `NavigateToPose()` 直至结束

### 4.3 常用参数

| 参数 | 说明 | 默认 |
|------|------|------|
| `--configuration_directory` | 配置根目录 | 必填 |
| `--start_x/y/yaw` | 起始位姿 | 0 |
| `--goal_x/y/yaw` | 目标位姿 | 必填 |
| `--use_bt` | BT 模式 | true |
| `--timeout_sec` | 超时（秒） | 120 |

### 4.4 构建说明

文档中常见：

```bash
cmake .. -DBUILD_TOOLS=ON -DBUILD_TASKS=ON
ninja autonomy_nav_test
```

**当前注意**：根 `CMakeLists.txt` 可能未定义 `BUILD_TOOLS`，且 `system/tools/CMakeLists.txt` 未必被 `add_subdirectory` 引用。若目标未生成，需检查 CMake 是否接入 `autonomy/system/tools/`。

### 4.5 环境变量

| 变量 | 说明 |
|------|------|
| `AUTONOMY_BT_PLUGIN_PATH` | BT 插件 `.so` 路径 |
| `GLOG_logtostderr=1` | 建议开启，便于查看日志 |

### 4.6 运行示例

```bash
export AUTONOMY_BT_PLUGIN_PATH=build/lib
export GLOG_logtostderr=1

./build/bin/autonomy_nav_test \
  --configuration_directory=config \
  --start_x=1 --start_y=1 --start_yaw=0 \
  --goal_x=5 --goal_y=5 --goal_yaw=0 \
  --use_bt=false \
  --timeout_sec=120
```

### 4.7 相关文档

- [04 Running · 离线导航测试](../04_Running/04_nav_test.md)
- [17 Tasks · 执行模式](../17_Tasks/06_execution_modes.md)
- 源码 README：`autonomy/system/tools/README.md`
