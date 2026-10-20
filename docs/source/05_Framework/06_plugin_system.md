# 6. 插件系统

PluginManager 运行时加载算法实现，与 Server 调度解耦。

## 6.1 类型

| 类型 | 接口 | 示例 |
|------|------|------|
| 全局规划器 | `planning::GlobalPlanner` | `navfn_planner`、`theta_star_planner` |
| 局部控制器 | `control::Controller` | `regulated_pure_pursuit` |
| BT 节点 | `BT::TreeNode` | `ComputePathToPose` |
| Costmap 层 | `costmap::Layer` | `static_layer`、`obstacle_layer` |

## 6.2 流程

```text
Server 启动 → 读 plugin 名 → PluginManager::CreateInstance
→ dlopen .so 或 REGISTER / 进程内注册 → Server 持有实例
```

## 6.3 开发步骤

1. 实现接口（如 `GlobalPlanner::createPlan`） 
2. `REGISTER_PLUGIN` / 导出宏 
3. CMake 编为 `.so` 
4. conf 注册 `plugin` 名与参数 

通信层 Plugin API：[03 Communication §7](../03_Communication/07_plugin.md)。 
模块细节：[08 Planning](../08_Planning/01_architecture.md) · [09 Control](../09_Control/04_controller_algorithms.md)。
