(async-grpc-rpc-lifecycle)=
# Rpc 生命周期

每次调用对应一个 `Rpc`：建连 → 读写 → 销毁。

### 3.1 四种类型

| Incoming | Outgoing | 类型 | Bridge |
|----------|----------|------|--------|
| Request | Response | Unary | Get* / Estop |
| Request | `Stream<Resp>` | Server Streaming | Navigate 等 |
| `Stream` | `Stream` | Bidi | Teleop |

### 3.2 Server Streaming

```text
NEW_CONNECTION → READ → OnRequest → WRITE* → FINISH → DONE
```

`Send`/`Finish` 入 `send_queue_`，EQ 串行写出。

### 3.3 回调

| 回调 | 用途 |
|------|------|
| `OnRequest` | 解析并调 Stub |
| `OnFinish` | 清理 |
| `Finish(Status)` | 失败结束 |
