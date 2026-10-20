(async-grpc-dual-queue)=
# 双队列

CQ（Completion）与 EQ（Event）分离：I/O 与 Handler 分属不同线程池。

### 2.1 职责

| 队列 | 线程参数 | 职责 |
|------|----------|------|
| CQ | `num_grpc_threads` | 网络 I/O、编解码 |
| EQ | `num_event_threads` | Handler 业务 |

同一次 RPC 内 Handler 单线程；跨 Rpc 共享状态用 `ExecutionContext`。

### 2.2 数据流

```text
Client → CQ(Next) → PushToEventQueue → EQ(Handle → OnRequest/Send/Finish)
Send/GetWriter → WRITE_NEEDED（直注 EQ，绕过 CQ）
```

### 2.3 事件

| Event | 含义 |
|-------|------|
| `NEW_CONNECTION` / `READ` | 建连 / 读请求 |
| `WRITE_NEEDED` / `WRITE` / `FINISH` / `DONE` | 写队列与结束 |

建议：$N_{\text{event}} \geq N_{\text{grpc}}$。
