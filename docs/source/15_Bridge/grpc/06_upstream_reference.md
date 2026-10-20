(async-grpc-upstream)=
# 上游参考

Vendoring：`autonomy/common/async_grpc/`，命名空间 `autonomy::common::async_grpc`。
| 资源 | 链接 |
|------|------|
| 上游仓库 | [cartographer-project/async_grpc](https://github.com/cartographer-project/async_grpc) |
| 典型场景 | Cloud 流式上传 |
| 最小示例 | 上游 README GetSquare |

升级 vendored async_grpc，或对照 Cartographer Cloud 行为。

### 与 Bridge 相关的点

| 主题 | 说明 |
|------|------|
| 线程模型 | 单 Rpc 回调同线程 |
| idle 连接 | 与 CQ 线程数、方法数相关 |
| `WRITE_NEEDED` | 写路径直注 EQ |
| Client | uplink/云侧可复用同一库做客户端 |

改 upstream 后跑 bridge unit tests + `GetCapabilities` 冒烟。
