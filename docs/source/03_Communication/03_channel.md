# 3. 通道 Channel

进程间持续数据流（类似 ROS 2 topic）。对端收不到，先核对 **channel 名 + 消息类型** 是否完全一致。 Channel 语义是「名称 + 消息类型」完全一致的持续流；对端收不到时先核对类型与 spelling，而不是怀疑序列化。同机默认同进程/SHM 路径，跨机才涉及 Fast DDS（见 [§14](14_fastdds.md)）。

## 传输与发现

| 场景 | 传输 |
|------|------|
| 同进程 | INTRA |
| 同机多进程 | SHM |
| 跨主机 | RTPS（可选，见 [§14](14_fastdds.md)） |

拓扑发现：本机文件总线（`/tmp/autolink_topology_events.log`）。跨机需 `AUTOLINK_ENABLE_FASTDDS`、`diff_host: RTPS`、相同 `AUTOLINK_DOMAIN_ID`、可达 `AUTOLINK_IP`。

## 发布与订阅

```cpp
auto writer = node->CreateWriter<Chatter>("channel/chatter");
writer->Write(msg);

node->CreateReader<Chatter>("channel/chatter",
    [](const std::shared_ptr<Chatter>& msg) { AINFO << msg->seq(); });
```

回调在 Scheduler 线程执行，勿长阻塞。业务消息：`commsgs` struct → 边界 `ToProto()`。POD 见 `pod_talker_listener.cpp`。

## ReaderConfig / QoS

| 字段 | 默认 | 含义 |
|------|------|------|
| `history` | `KEEP_LAST` | 保留最近 N 条 |
| `depth` | `1` | 历史深度 |
| `reliability` | `RELIABLE` | 可靠传输 |
| `durability` | `VOLATILE` | 不保留历史给晚加入者 |
| `pending_queue_size` | 默认常量 | 待处理回调队列 |

控制环建议 `pending_queue_size = 1`：

```cpp
autolink::ReaderConfig cfg;
cfg.channel_name = "/plan";
cfg.pending_queue_size = 1;
node->CreateReader<Path>(cfg, callback);
```

## Rate

```cpp
Rate rate(10.0); // 10 Hz
while (autolink::OK()) {
    writer->Write(msg);
    rate.Sleep();
}
```

## CLI

| 工具 | 用途 |
|------|------|
| `autolink_channel` | 列出 channel 与类型 |
| `autolink_monitor` | 实时流量 |
| `autolink_recorder` | `record -a -o x.record` / `play -f x.record` |
