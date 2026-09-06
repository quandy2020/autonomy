# 3. 通道 Channel

Channel 是 autolink 的 **持续数据流** 抽象，对标 ROS 2 topic。Writer 发布、Reader 订阅；匹配条件为 **channel 名一致 + 消息类型一致**。

| 本文 §3 | 相关文档 |
|---------|----------|
| 发布订阅 | [§0 指南](00_guide.md) · [§2 节点 Node](02_node.md) · [§10 组件 Component](10_component.md) · [§12 调度 Scheduler](12_scheduler.md) |

---

## 3.1 传输与发现

默认传输：同进程 INTRA、同机多进程 SHM；拓扑发现为本机文件总线（`/tmp/autolink_topology_events.log`）。
跨机 Channel 为**可选**能力：编译打开 `AUTOLINK_ENABLE_FASTDDS`，配置 `diff_host: RTPS`，
双方设置可达 `AUTOLINK_IP` 与相同 `AUTOLINK_DOMAIN_ID`（默认 80），依赖 Fast DDS SIMPLE 多播做端点匹配。
完整跨机拓扑图（ChangeMsg over RTPS）当前未提供。

| 场景 | 传输 |
|------|------|
| 同进程 | INTRA（内存直传） |
| 同机多进程 | SHM（共享内存） |
| 跨主机 | RTPS（可选；需 Fast DDS） |

拓扑发现（L2）始终为本机文件总线；可选 RTPS 数据面内部另有 DDS 端点发现，不属于 Autolink L2。匹配完成后业务数据不经发现通道。

---

## 3.2 消息类型

```protobuf
message Chatter {
  uint64 timestamp = 1;
  uint64 lidar_timestamp = 2;
  uint64 seq = 3;
  bytes content = 4;
}
```

Autonomy 业务消息为 `autonomy::commsgs::*` struct，跨进程边界 `ToProto()` 后走 Channel。亦支持 **POD 包装类型**（见 `pod_talker_listener.cpp` + `pod_packet.hpp`），适合高频小结构体。

---

## 3.3 发布与订阅

**发布**（`talker.cpp`）：

```cpp
auto node = autolink::CreateNode("talker");
auto writer = node->CreateWriter<Chatter>("channel/chatter");

while (autolink::OK()) {
    auto msg = std::make_shared<Chatter>();
    msg->set_seq(seq++);
    msg->set_content("Hello, autolink!");
    writer->Write(msg);   // 同步写入传输层
    rate.Sleep();
}
```

**订阅**（`listener.cpp`）：

```cpp
void MessageCallback(const std::shared_ptr<Chatter>& msg) {
    AINFO << "Received message seq-> " << msg->seq();
    AINFO << "msgcontent->" << msg->content();
}

auto reader = node->CreateReader<Chatter>("channel/chatter", MessageCallback);
```

回调在 **Scheduler 线程**执行，勿长时间阻塞；重逻辑移交工作线程或 Component `Proc`。

---

## 3.4 ReaderConfig 与 QoS

`ReaderConfig` 默认值（`node_channel_impl.hpp`）：

| 字段 | 默认 | 含义 |
|------|------|------|
| `qos_profile.history` | `KEEP_LAST` | 只保留最近 N 条 |
| `qos_profile.depth` | `1` | 历史深度 |
| `qos_profile.reliability` | `RELIABLE` | 可靠传输 |
| `qos_profile.durability` | `VOLATILE` | 不保留晚加入订阅者之前的数据 |
| `pending_queue_size` | `DEFAULT_PENDING_QUEUE_SIZE` | 待处理回调队列长度 |

控制 / 规划环建议 **只处理最新一帧**：

```cpp
autolink::ReaderConfig cfg;
cfg.channel_name = "/plan";
cfg.pending_queue_size = 1;
node->CreateReader<Path>(cfg, [](const std::shared_ptr<Path>& p) {
    // 轻量处理
});
```

队列满时旧消息被丢弃，避免延迟累积。

亦可通过 `proto::RoleAttributes` 精细设置 channel 名与 QoS：

```cpp
proto::RoleAttributes attr;
attr.set_channel_name("/sensor/imu");
attr.mutable_qos_profile()->set_depth(10);
node->CreateWriter<Imu>(attr);
```

---

## 3.5 POD 消息

`pod_talker_listener.cpp` 演示四步：

1. 定义 trivially copyable POD 结构体  
2. 包装类实现 `TypeName`、`SerializeToArray` / `ParseFromArray`  
3. `CreateWriter<PodPacket>` / `CreateReader<PodPacket>`  
4. 填充字段后 `Write`

```cpp
auto writer = node->CreateWriter<PodPacket>("channel/pod_demo");
auto reader = node->CreateReader<PodPacket>("channel/pod_demo", OnPodMessage);

auto msg = std::make_shared<PodPacket>();
msg->pod.seq = seq;
msg->pod.value = static_cast<double>(seq) * 0.5;
msg->pod.timestamp_ns = Time::Now().ToNanosecond();
writer->Write(msg);
```

单进程验证：`./autolink_example_pod_talker_listener`；跨进程：`pod_talker` + `pod_listener`。

---

## 3.6 Rate 定频

C++ `Rate`（Hz）：

```cpp
autolink::Rate rate(1.0);
while (autolink::OK()) {
    writer->Write(msg);
    rate.Sleep();
}
```

Python（`py_talker.py`）：

```python
rate = autolink_time.Rate(1.0)
while not autolink.is_shutdown():
    writer.write(msg)
    rate.sleep()
```

---

## 3.7 数据路径

发现匹配完成后，业务数据不经发现通道，按拓扑自动选择传输后端。

<div class="comm-flow-diagram">
<div class="comm-flow-header">
  <span class="comm-flow-badge">Channel</span>
  <span class="comm-flow-title">Write → 传输 → pending_queue → 调度 → 回调</span>
  <span class="comm-flow-sub">与 §1.2 内部分层 L3–L4 对应；Component 在回调处接入 DataVisitor</span>
</div>

<div class="comm-flow-pipeline comm-flow-pipeline--chain">
  <div class="comm-flow-step comm-flow-step-client">
    <span class="comm-flow-step-label">发布</span>
    <span class="comm-flow-step-title">Writer::Write</span>
  </div>
  <div class="comm-flow-link comm-flow-link--labeled">
    <span class="comm-flow-link-label">INTRA · SHM · RTPS（可选）</span>
    <span class="comm-flow-link-arrow" aria-hidden="true">→</span>
  </div>
  <div class="comm-flow-step comm-flow-step-mid">
    <span class="comm-flow-step-label">订阅</span>
    <span class="comm-flow-step-title">pending_queue</span>
  </div>
  <div class="comm-flow-link"><span class="comm-flow-link-arrow" aria-hidden="true">→</span></div>
  <div class="comm-flow-step comm-flow-step-mid">
    <span class="comm-flow-step-title">Scheduler</span>
  </div>
  <div class="comm-flow-link"><span class="comm-flow-link-arrow" aria-hidden="true">→</span></div>
  <div class="comm-flow-step comm-flow-step-server">
    <span class="comm-flow-step-title">回调 / DataVisitor</span>
  </div>
</div>

<table class="comm-flow-legend">
  <tr><th>模式</th><th>回调目标</th></tr>
  <tr><td>Binary</td><td>用户 <code>CreateReader</code> 回调</td></tr>
  <tr><td>Component</td><td><code>DataVisitor</code> → 多路齐备 → <code>Proc</code>（[§1.4](01_architecture.md#14-component-数据流)）</td></tr>
</table>

<div class="comm-flow-foot">回调在 Scheduler 工作线程执行，保持轻量；重逻辑移交业务线程。</div>
</div>

---

## 3.8 Python API

**发布**（`py_talker.py`）：

```python
test_node = autolink.Node("node_name1")
writer = test_node.create_writer("channel/chatter", Chatter, 6)
rate = autolink_time.Rate(1.0)
while not autolink.is_shutdown():
    msg = Chatter()
    msg.seq = g_count
    msg.content = b"I am python talker."
    writer.write(msg)
    g_count += 1
    rate.sleep()
```

**订阅**（`py_listener.py`）：

```python
def callback(data):
    print("seq:", data.seq, "content:", data.content)

test_node = autolink.Node("listener")
test_node.create_reader("channel/chatter", Chatter, callback)
test_node.spin()
```

---

## 3.9 CLI 工具

```bash
autolink_channel list          # 列出活跃 channel 与类型
autolink_monitor               # 实时消息监控
autolink_recorder record -a -o x.record   # 录包
autolink_recorder play -f x.record        # 回放
```

录包示例源码：`autolink/examples/cpp/record.cpp`。

---

**导航**：[← §2 节点 Node](02_node.md) · [§0 指南](00_guide.md) · [§4 服务 Service →](04_service.md)
