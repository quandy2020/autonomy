# 0. 框架指南

## 背景

改控制器参数却「不生效」、插件加载失败、不知道某个域该起哪个进程——往往卡在 **`libautonomy` 装配层**：Server 边界、Lua/pb 配置管线、插件注册名。通信 API 在 [03 Communication](../03_Communication/00_guide.md)；具体规划/控制算法在 08/09 章。

## 思路

框架解决的是 **把各域装成可运行系统**：

- 多进程里每个域的 **Server**（Map / Planner / Controller / Task…）管生命周期、接口与线程；算法多在插件或子目录。  
- **配置分层**：系统 launch / system conf → 域级 `autonomy/<域>/conf/` → 插件 options。  
- **`planner_plugins` / `controller_plugins` 字符串** 必须和注册名一致，和目录名不一定相同。  
- 导航不只 planning/control：`sensor` / `vehicle` / `audio` 等支撑模块见 [09 Sensor/Vehicle/Audio](09_sensor_vehicle_audio.md)。

若只改一个控制器参数，可直接去 [09 Control](../09_Control/00_guide.md)；若要 **新加 Server 或插件类型**，从 [05 Server](05_module_servers.md)、[06 插件](06_plugin_system.md) 开始。


## 步骤

### 1. 按问题选页

| 文档 | 解决什么 |
|------|----------|
| [01 概览](01_overview.md) | 框架组件清单 |
| [02 快速开始](02_quickstart.md) | 最小装配示例 |
| [03 架构](03_architecture.md) | 分层与数据流 |
| [04 配置](04_configuration.md) | Options / Lua / pb 关系 |
| [05 Server](05_module_servers.md) | 各 Server 职责与入口 |
| [06 插件](06_plugin_system.md) | 插件表、解析规则 |
| [07 消息集成](07_commsgs_integration.md) | commsgs / Autolink 接缝 |
| [08 综述](08_survey.md) | 能力对照 |
| [09 Sensor/Vehicle/Audio](09_sensor_vehicle_audio.md) | 传感整理、底盘桥、麦阵 |

**搞清该起哪个进程** → [05](05_module_servers.md) + [Running · 多进程](../04_Running/03_autonomy_process.md)。  
**新参数不生效** → [04](04_configuration.md)。  
**插件加载失败** → [06](06_plugin_system.md)。

### 2. 动手时记住

1. **Server ≠ 算法实现** — 排错先分清「Server 没起来」还是「插件算错」。  
2. **改错配置层会「改了不生效」** — 确认 launch 加载的是哪一份 conf。  
3. **上游 channel / TF 未就绪** — Server Start 后也可能长时间无业务输出。

### 3. 常见坑

| 现象 | 先查 |
|------|------|
| Options 是空的 | 是否走了正确的 LoadOptions / Lua 入口 |
| 插件找不到 | 库路径；注册宏；plugins 字符串 |
| Server Start 后无输出 | 上游 channel 是否有人发；TF 是否就绪 |
