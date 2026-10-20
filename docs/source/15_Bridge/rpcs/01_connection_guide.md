(bridge-connection-guide)=
# 接入与验证

调 `automsgs.rpcs.*`。用例索引：[12](12_integration_tests.md)。推荐 `automsgs/tools/cli/rpc-cli.py`。

### 1.1 环境

| 参数 | 默认 | 说明 |
|------|------|------|
| 地址 | `127.0.0.1:5005` | [§1](../01_options.md) |
| Proto | `automsgs/proto/rpcs/<domain>.proto` | `-import-path` = 仓库根 |

```bash
export REPO=/path/to/autonomy
export BRIDGE=127.0.0.1:5005
export PROTO_OPTS="-import-path $REPO -proto automsgs/proto/rpcs/system.proto"
export SVC=automsgs.rpcs.system.SystemService
```

### 1.2 服务发现

```bash
grpcurl -plaintext $PROTO_OPTS $BRIDGE list
grpcurl -plaintext $PROTO_OPTS $BRIDGE list $SVC
grpcurl -plaintext $PROTO_OPTS $BRIDGE describe $SVC.GetCapabilities
```

| 命令 | 判定 |
|------|------|
| `list` | 含 `automsgs.rpcs.*.*Service` |
| `describe` | 请求/响应类型正确 |

### 1.3 冒烟

```bash
grpcurl -plaintext $PROTO_OPTS -d '{}' "$BRIDGE" "$SVC/GetCapabilities"
```
