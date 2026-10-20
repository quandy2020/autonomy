# 3. 通道 Channel

持续数据流抽象，类似 ROS 2 topic。匹配：**channel 名一致 + 消息类型一致**。

## 3.1 传输与发现

| 场景 | 传输 |
|------|------|
| 同进程 | INTRA |
| 同机多进程 | SHM |
| 跨主机 | RTPS（可选，见 [§14](14_fastdds.md)） |

拓扑发现：本机文件总线（`/tmp/autolink_topology_events.log`）。跨机需 `AUTOLINK_ENABLE_FASTDDS`、`diff_host: RTPS`、相同 `AUTOLINK_DOMAIN_ID`、可达 `AUTOLINK_IP`。

## 3.2 发布与订阅

```cpp
auto writer = node->CreateWriter<Chatter>("channel/chatter");
writer->Write(msg);

node->CreateReader<Chatter>("channel/chatter",
    [](const std::shared_ptr<Chatter>& msg) { AINFO << msg->seq(); });
```

回调在 Scheduler 线程执行，勿长阻塞。业务消息：`commsgs` struct → 边界 `ToProto()`。POD 见 `pod_talker_listener.cpp`。

## 3.3 ReaderConfig / QoS

| 字段 | 默认 | 含义 |
|------|------|------|
| `history` | `KEEP_LAST` | 保留最近 N 条 |
| `depth` | `1` | 历史深度 |
| `reliability` | `RELIABLE` | — |
| `durability` | `VOLATILE` | — |
| `pending_queue_size` | 默认常量 | 待处理回调队列 |

控制环建议 `pending_queue_size = 1`：

```cpp
autolink::ReaderConfig cfg;
cfg.channel_name = "/plan";
cfg.pending_queue_size = 1;
node->CreateReader<Path>(cfg, callback);
```

## 3.4 Rate

```cpp
Rate rate(10.0); // 10 Hz
while (autolink::OK()) {
    writer->Write(msg);
    rate.Sleep();
}
```

## 3.5 CLI

| 工具 | 用途 |
|------|------|
| `autolink_channel` | 列出 channel 与类型 |
| `autolink_monitor` | 实时流量 |
| `autolink_recorder` | `record -a -o x.record` / `play -f x.record` |
