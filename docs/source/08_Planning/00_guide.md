# 0. Planning 指南

## 背景

规划总是超时、有路径却贴障——一半情况是 **costmap 不健康**，另一半才是算法选型。本章讲 **全局路径规划**：输入起终点 + Costmap2D，输出几何 `Path`（无时间戳轨迹）；局部跟踪在 [09 Control](../09_Control/00_guide.md)。

## 思路

```text
Goal → PlannerServer → Path → ControllerServer
         ↑ Costmap2D（常共享或同进程配置）
```

- 角色对齐 Nav2 planner；**不要在 planning 里找 `cmd_vel`**。  
- **`default_planner_id` / `planner_plugins`** 字符串必须和注册名一致。  
- 频率通常 1–5 Hz 或事件触发，不要按控制环频率去调规划。  
- costmap 问题先查 [07 Map](../07_Map/00_guide.md)。


## 步骤

### 1. 按问题选页

| 文档 | 解决什么 |
|------|----------|
| [01 架构](01_architecture.md) | PlannerServer、结果码、时序 |
| [02 算法总览](02_planner_algorithms.md) | 三种插件怎么选 |
| [03 NavFn](03_navfn.md) | 默认势场类 |
| [04 Dijkstra](04_dijkstra.md) | 经典最短路 |
| [05 Theta*](05_theta_star.md) | 任意角 |
| [06 综述](06_survey.md) | 对照与边界 |

**默认室内网格** → NavFn（[03](03_navfn.md)）。  
**要更直的路径** → Theta*（[05](05_theta_star.md)）。  
**失败码含义** → [01](01_architecture.md)。

### 2. 最短示例

```lua
default_planner_id = "navfn_planner"
planner_plugins = { "navfn_planner", "dijkstra_planner", "theta_star_planner" }
```

```cpp
auto server = std::make_shared<PlannerServer>(CreateOptions("config"));
auto path = server->GetPlan(start, goal, "navfn_planner", []{ return false; });
```

时序与结果码：[01](01_architecture.md)。算法对比：[02](02_planner_algorithms.md)。

### 3. 常见坑

| 现象 | 先查 |
|------|------|
| 总是超时 | costmap 未 current；地图太大；起点在障碍里 |
| 有路径但很难跟 | 路径贴障；膨胀与 control 配合 |
| 换了 planner 无效 | plugins 字符串；是否重新加载 conf |
