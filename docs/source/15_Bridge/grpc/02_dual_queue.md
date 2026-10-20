(async-grpc-dual-queue)=
# 双队列

| 队列 | `GrpcOptions` | 职责 |
|------|---------------|------|
| CQ | `num_grpc_threads` | 收发包、编解码 |
| EQ | `num_event_threads` | `OnRequest`、写队列、`Finish` |
同一次 RPC 内回调在同一线程；**跨 RPC** 共享 Muxer/Hub 需要各自锁。

**为什么分 CQ/EQ：** 网络线程只做 I/O，业务在 EQ 里跑，避免 Handler 里阻塞读 socket。

### 数据流

```text
Client → CQ(Next) → PushToEventQueue → EQ(OnRequest / Send / Finish)
GetWriter() → WRITE_NEEDED（直投 EQ，不经 CQ 读路径）
```

### 调参

建议 $N_{\text{event}} \geq N_{\text{grpc}}$。Action 阻塞用 `WorkScheduler` 卸到 worker，勿在 EQ 里 sleep。

### 常见坑

| 现象 | 说明 |
|------|------|
| EQ 饿死 | grpc 线程过多 event 过少 |
| 跨线程裸 Send | 竞态；用 GetWriter |
| Handler 里长计算 | 拖慢所有 RPC；卸荷到 WorkScheduler |
