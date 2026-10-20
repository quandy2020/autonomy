(async-grpc-rpc-lifecycle)=
# Rpc 生命周期

每次调用一个 `Rpc` 实例：建连 → 读请求 → 写响应(流) → Finish → DONE。

Stream 半开半关、Finish 顺序错乱，或在写 Bidi 客户端。

### 四种类型

| Incoming | Outgoing | Bridge 例子 |
|----------|----------|-------------|
| Request | Response | GetCapabilities、Estop |
| Request | Stream | Navigate、Follow |
| Stream | Stream | Teleop Velocity |
| Stream | Response | （少用） |

### Server Streaming 顺序

```text
NEW_CONNECTION → READ → OnRequest → WRITE* → FINISH → DONE
```

`Send` 入队，EQ 串行写出；`Finish` 后不应再 `Send`。

### 与 Bridge 语义

- Navigate：**ARRIVED 不一定 Finish**（proto 允许保持 stream）。  
- 拒绝/busy：可能首帧即带 `common.Status` 然后 Finish。  
- 客户端断连：触发取消路径，别当任务成功。

### 回调

| 回调 | 用途 |
|------|------|
| `OnRequest` | 解析、gate、调 Stub |
| `OnFinish` | 清理 session |
| `Finish(Status)` | gRPC 层失败 |
