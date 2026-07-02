# 6. Node 与 Channel

本章深入 Node、Writer、Reader 的实现要点与最佳实践。

## 6.1 Node 内部结构

`Node` 是对外的轻量门面，内部组合两个实现类：

```
Node
├── NodeChannelImpl   # Writer / Reader / 通道属性
└── NodeServiceImpl   # Service / Client
```

创建接口：

```cpp
std::shared_ptr<Node> CreateNode(
    const std::string& node_name,
    const std::string& name_space = "");
```

**前置条件**：`autolink::Init()` 已成功调用，否则返回 `nullptr`。

**命名规则**：

- `node_name` 在全局拓扑中唯一
- 带 namespace 时完整标识为 `/namespace/node_name`
- 同一 Node 可创建多个 Writer/Reader，但 channel 角色 ID 不可冲突

## 6.2 Writer 详解

### 6.2.1 创建方式

| 重载 | 场景 |
|------|------|
| `CreateWriter<T>(channel_name)` | 最常用，默认 QoS |
| `CreateWriter<T>(RoleAttributes)` | 自定义 depth、mps、类型名 |

### 6.2.2 写入路径

```cpp
template <typename MessageT>
bool Writer<MessageT>::Write(const MessageT& msg);
bool Writer<MessageT>::Write(const std::shared_ptr<MessageT>& msg);
```

内部流程：

1. 检查 `IsInit()` 与是否有活跃 Reader（可选优化）
2. 通过 `Transmitter<MessageT>` 序列化并发送
3. `ChannelManager` 已注册的 Receiver 收到通知

### 6.2.3 实用 API

| 方法 | 说明 |
|------|------|
| `HasReader()` | 是否存在订阅者（用于降频空发） |
| `GetChannelName()` | 通道名 |
| `Shutdown()` | 主动关闭，反注册拓扑 |

## 6.3 Reader 详解

### 6.3.1 回调模型

```cpp
using CallbackFunc = std::function<void(const std::shared_ptr<MessageT>&)>;
```

回调在 **Scheduler 调度的 CRoutine** 或 **Receiver 线程** 中执行（取决于配置），因此：

- 回调应尽量轻量，重计算应投递到自有线程池
- 避免在回调中阻塞等待其他 Reader（易死锁）

### 6.3.2 ReaderConfig

```cpp
struct ReaderConfig {
    std::string channel_name;
    uint32_t pending_queue_size = 1;
    // ...
};
```

| `pending_queue_size` | 适用 |
|----------------------|------|
| 1 | 控制指令、最新状态（只关心最新） |
| 10+ | 传感器数据（允许短暂积压） |

### 6.3.3 无回调轮询

Component 框架通过 `DataVisitor` + `Observe()` 拉取多路输入，而非每路单独回调。自定义节点一般使用回调即可。

## 6.4 Channel 与 RoleAttributes

`RoleAttributes`（proto）描述通信角色元数据：

| 字段 | 含义 |
|------|------|
| `channel_name` | 通道名，如 `/sensor/imu` |
| `message_type` | 类型全名，用于发现匹配 |
| `qos_profile` | depth、可靠性等 |
| `node_name` / `id` | 所属节点与角色 ID |

**匹配规则**：相同 `channel_name` + 兼容 `message_type` 的 Writer/Reader 建立连接。

## 6.5 多订阅者与多发布者

```
         ┌── Reader A (callback A)
Writer ──┼── Reader B (callback B)
         └── Reader C

Writer 1 ──┐
Writer 2 ──┼── Reader X
Writer 3 ──┘
```

- 多 Writer 时 Reader 收到**多源**消息，无内建时间戳同步
- 多 Reader 时 Writer 一次 Write 广播至所有 Receiver
- 需要「每消息一次处理」时，在应用层去重或选主

## 6.6 DataVisitor 与 Component 输入

`DataVisitor<M0,M1,...>` 缓存多 channel 最新消息，当**全部**输入到齐时触发 `Component::Proc`：

```cpp
class FusionComponent : public Component<Imu, Odometry> {
    bool Proc(const std::shared_ptr<Imu>& imu,
              const std::shared_ptr<Odometry>& odom) override;
};
```

DAG 中配置各 reader 的 channel 与 QoS，mainboard 启动时自动连线。

## 6.7 Channel 命名约定（Autonomy）

| 前缀 | 示例 | 说明 |
|------|------|------|
| `/sensor/` | `/sensor/imu` | 传感器原始数据 |
| `/map` | `/map` | 静态地图 |
| `/plan` | `/plan` | 全局路径 |
| `/cmd_vel` | `/cmd_vel` | 速度指令 |
| `/tf` | `/tf` | 坐标变换 |

与 ROS 2 话题命名保持一致，便于 Bridge 映射。

## 6.8 性能建议

1. **同进程通信用 INTRA**：避免 SHM 拷贝
2. **大消息用 SHM**：点云、地图优先共享内存
3. **控制用小 depth**：避免陈旧 cmd 被执行
4. **避免在回调里分配大对象**：复用 buffer 或对象池
5. **高频 Writer 检查 `HasReader()`**：无订阅时可跳过计算

## 6.9 调试

```bash
# 列出所有 channel
autolink_channel list

# 监听某 channel 消息
autolink_monitor --channel=/sensor/imu
```

## 6.10 常见问题

| 问题 | 解答 |
|------|------|
| Writer 写了但 Reader 收不到 | 检查 channel 名、消息类型、是否同域发现 |
| 回调频率低于发布频率 | `pending_queue_size=1` 会丢中间帧 |
| 同进程两节点不通 | 确认已 Init，且 channel 名完全一致 |
| protobuf 解析失败 | Writer/Reader 模板类型不一致 |
