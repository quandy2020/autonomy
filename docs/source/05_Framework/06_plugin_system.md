# 6. 插件系统

规划器、控制器、代价地图层、BT 节点等算法通过 PluginManager 加载，与 Server 生命周期分开。

算法与 Server 生命周期解耦：conf 里 id 映射到类名与 `.so`。换 planner/controller 通常只改 pb；BT 节点、costmap layer 是另一套插件表与 loader。

## 类型

| 类型 | 接口 | 示例 |
|------|------|------|
| 全局规划器 | `planning::GlobalPlanner` | `navfn_planner`、`theta_star_planner` |
| 局部控制器 | `control::Controller` | `regulated_pure_pursuit` |
| BT 节点 | `BT::TreeNode` | `ComputePathToPose` |
| Costmap 层 | `costmap::Layer` | `static_layer`、`obstacle_layer` |

## 流程

```text
Server 启动 → 读 plugin 名 → PluginManager::CreateInstance
→ dlopen .so 或 REGISTER / 进程内注册 → Server 持有实例
```

## 开发步骤

1. 实现接口（如 `GlobalPlanner::createPlan`）
2. `REGISTER_PLUGIN` / 导出宏
3. CMake 编为 `.so`
4. conf 注册 `plugin` 名与参数

通信层 Plugin API：[03 Communication §7](../03_Communication/07_plugin.md)。  
模块细节：[08 Planning](../08_Planning/01_architecture.md) · [09 Control](../09_Control/04_controller_algorithms.md)。
