# 0. Communication 指南

## 背景

写节点、起 launch、对 channel 却收不到数据，问题多半不在业务算法，而在 **Autolink** 初始化顺序、环境变量或 channel 契约。本章只讲进程间通信运行时（角色接近 Cyber RT / ROS 2 通信层）；消息类型定义在 [14 Commsgs](../14_Commsgs/00_guide.md)。

## 思路

Autolink 的心智模型可以压成四条：

- 进程里先 **Init**，再建 **Node**，再挂 Writer/Reader、Service/Client、Action、Parameter。
- **Channel** 靠「名字 + 消息类型」匹配，不是「同进程碰巧能通」。
- 同机多进程必须共享 **`AUTOLINK_PATH`**（含 `conf/autolink.pb.conf`）。
- 回调里别做重 IO / 重计算；卡住一条回调会拖整条链路。

示例代码：`autolink/examples/cpp/`、`autolink/examples/python/`。

```bash
source autolink/autolink/setup.bash
export AUTOLINK_PATH=<含 conf/autolink.pb.conf 的目录>
```


## 步骤

### 1. 按问题选页

| 文档 | 解决什么 |
|------|----------|
| [01 架构](01_architecture.md) | 分层与生命周期 |
| [02 Node](02_node.md) | 建节点 |
| [03 Channel](03_channel.md) | 收发 |
| [04 Service](04_service.md) | 同步 RPC |
| [05 Action](05_action.md) | 长时任务 |
| [06 Parameter](06_parameter.md) | 运行时参数 |
| [07 Plugin](07_plugin.md) | 插件加载 |
| [08 Log](08_log.md) | 日志 |
| [09 Launch](09_launch.md) | 多进程编排 |
| [10 Component](10_component.md) | 可 launch 组件 |
| [11 Timer](11_timer.md) | 定时器 |
| [12 Scheduler](12_scheduler.md) | 调度 |
| [13 综述](13_survey.md) | 与 ROS/其它栈对照 |
| [14 FastDDS](14_fastdds.md) | 跨机 RTPS（可选） |

**两进程发一条消息** → [02](02_node.md) + [03](03_channel.md)，对着 examples 改 channel 名和 proto。  
**加可 launch 组件** → [10](10_component.md) → [09 Launch](09_launch.md)。  
**从 ROS 2 迁过来** → [04](04_service.md)–[06](06_parameter.md) 语义对照；勿假定 QoS 与 DDS 完全同款。

### 2. 动手时记住

1. **先 `Init()`，再造 Node** — 顺序反了会很隐蔽地挂。  
2. **`node_name` 全局唯一** — 多进程复制粘贴最容易撞名。  
3. **Channel 两端类型必须一致** — 名字对了类型错了等于没连上。  
4. **车上长期跑的是 dag + launch**，不是临时 `main` 小例子。  
5. **本章只管「怎么传」** — 类型在 commsgs / [14 Commsgs](../14_Commsgs/00_guide.md)。

### 3. 常见坑

| 现象 | 先查 |
|------|------|
| 对端永远无数据 | `AUTOLINK_PATH`；channel 名；消息类型是否同一份 proto |
| 进程起来立刻退出 | 是否未 `Init`；node 重名 |
| 偶发卡死 | 回调里是否阻塞 |
| launch 起不来 | dag / launch 路径；Component 库是否在 `LD_LIBRARY_PATH` |
