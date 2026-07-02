# 6. 进程内离线仿真（nav_test）

本文详述 `autonomy/system/tools/nav_test` 的设计、用法与实现细节。

---

## 6.1 定位

`autonomy_nav_test` 在**不依赖 ROS** 的环境下跑通：

```
Autonomy → Map / Planner / Controller / Navigator(BT)
```

并用差速模型仿真里程计与 TF，形成**控制闭环**。

| 对比工具 | 覆盖 |
|----------|------|
| **nav_test** | 全栈 + BT + 恢复行为 |
| `autonomy_planning_test` | 仅规划 |
| `autonomy_controller_test` | 规划 + 控制，无 BT |
| `system/main` | 仅 Start，不发目标 |

---

## 6.2 核心函数

### 6.2.1 IntegrateDiffDrive

```cpp
void IntegrateDiffDrive(const Twist& cmd, double dt,
                        double& x, double& y, double& yaw) {
    const double v = cmd.linear.x;
    const double w = cmd.angular.z;
    x += v * std::cos(yaw) * dt;
    y += v * std::sin(yaw) * dt;
    yaw = NormalizeAngle(yaw + w * dt);
}
```

数学推导见 [03_math.md §3.1](03_math.md#31-差速驱动运动学)。

### 6.2.2 PublishSimState

向 `ControllerServer::UpdateOdometry` 注入里程计，并向 `transform::Buffer` 发布 `global_frame → base_frame` TF。

### 6.2.3 RunNavTest 主流程

1. 加载 `config/autonomy.lua`
2. `CreateAutonomy` → `Start()` → `Configure()`
3. 设置初始位姿（odom + TF）
4. 启动仿真线程（周期 `sim_dt`）
5. 主线程 `NavigateToPose(goal)` 阻塞直至完成

---

## 6.3 导航模式

| `--use_bt` | 行为 |
|------------|------|
| `true`（默认） | `Task::StartNavigateToPose` → BT（Plan / Follow / Recovery） |
| `false` | `NavigateDirectToPose`：Plan + Follow 循环 |

---

## 6.4 构建与运行

```bash
cmake .. -DBUILD_TOOLS=ON -DBUILD_TASKS=ON
ninja autonomy_nav_test

export AUTONOMY_BT_PLUGIN_PATH=/workspace/autonomy/build/lib
export GLOG_logtostderr=1

./bin/autonomy_nav_test \
  --configuration_directory=/workspace/autonomy/src/autonomy/config \
  --start_x=1 --start_y=1 --goal_x=5 --goal_y=5 \
  --use_bt=true --timeout_sec=120
```

---

## 6.5 数据流图

```text
autonomy_nav_test
  ├─ CreateAutonomy → Start → Configure
  ├─ [sim thread @ sim_dt]
  │     GetLastControlCommand()
  │       → IntegrateDiffDrive()
  │       → UpdateOdometry + map→base_link TF
  └─ NavigateToPose
        ├─ use_bt=true  → BT (Plan/Follow/Recovery…)
        └─ use_bt=false → NavigateDirectToPose
```

---

## 6.6 与 Autonomy API 的配合

```cpp
// autonomy.hpp
Twist GetLastControlCommand() const;  // 仿真线程读取
void NavigateToPose(const PoseStamped& goal);
```

仿真线程与主线程通过 `Autonomy` 内部状态与控制服务器通信，无需 ROS topic。

---

## 6.7 限制与扩展

| 限制 | 说明 |
|------|------|
| 无传感器仿真 | 依赖静态地图，无动态障碍 |
| 理想运动学 | 无打滑、延迟、噪声 |
| 仅差速模型 | 全向/Ackermann 需扩展 `Integrate*` |

**扩展建议**：

- 加里程计噪声：$\mathbf{x}_{k+1} = f(\mathbf{x}_k, u_k) + \mathbf{w}_k$
- 接入 `Vehicle::ApplyCommand` 统一接口
- 从文件回放 `cmd_vel` 轨迹做开环测试

---

## 6.8 常见问题

| 现象 | 处理 |
|------|------|
| BT 插件 load 失败 | 设置 `AUTONOMY_BT_PLUGIN_PATH` |
| 规划失败 | 起终点需在 `map.pgm` 自由空间 |
| `no robot pose` | 确认 Controller `Start()` 与初始 odom 注入 |
| 超时 | 增大 `--timeout_sec` 或缩短路径 |

完整说明见 `autonomy/system/tools/README.md`。
