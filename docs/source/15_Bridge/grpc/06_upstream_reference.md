(async-grpc-upstream)=
# 上游参考

Vendoring：`autonomy/common/async_grpc/`，命名空间 `autonomy::common::async_grpc`。

| 资源 | 链接 |
|------|------|
| 仓库 | [cartographer-project/async_grpc](https://github.com/cartographer-project/async_grpc) |
| 场景 | Cartographer Cloud 流式上传 |
| 最小 Unary | README GetSquare |

### 要点

| 主题 | 说明 |
|------|------|
| 线程保证 | 单次 RPC 同线程 → Handler 不必 thread-safe |
| idle Rpc | $N_{\text{cq}} \times M_{\text{methods}}$ |
| `WRITE_NEEDED` | 直注 EQ |
| 测试 | `RpcHandlerTestServer` |
| Client / Retry | uplink 可复用 |
