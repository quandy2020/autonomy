(bridge-connection-guide)=
# 接入与验证

系统测试与研发联调 **`automsgs.rpcs.*`** 的标准接入流程。完整用例库见 **[13 集成测试参考](13_integration_tests.md)**。推荐优先用 `automsgs/tools/cli/rpc-cli.py`。

读完本章后按 [02 服务概述](02_service_overview.md) → [03 公共消息类型](03_common_types.md) 继续。

## 1.1 环境配置

| 参数 | 默认值 | 说明 |
|------|--------|------|
| 地址 | `127.0.0.1:5005` | 机载 [§1 Bridge 配置](../01_options.md) |
| 服务 | 如 `automsgs.rpcs.system.SystemService` | 按域选择 |
| Proto | `automsgs/proto/rpcs/<domain>.proto` | `-import-path` 指向仓库根 |

```bash
export REPO=/path/to/autonomy          # 改为本机仓库路径
export BRIDGE=127.0.0.1:5005
export PROTO_OPTS="-import-path $REPO -proto automsgs/proto/rpcs/system.proto"
export SVC=automsgs.rpcs.system.SystemService
new_goal_id() { echo "test-$(date +%s)-$RANDOM"; }

grpc_call() {
  grpcurl -plaintext $PROTO_OPTS -d @- "$BRIDGE" "$SVC/$1"
}
```

**安装 grpcurl**：`brew install grpcurl` 或 `go install github.com/fullstorydev/grpcurl/cmd/grpcurl@latest`

## 1.2 服务发现

确认 Bridge 可达、Proto 路径正确（无需机载任务）。

| 命令 | 判定 |
|------|------|
| `list` | 输出含 `automsgs.rpcs.*.*Service` |
| `list $SVC` | 该域全部 RPC 方法 |
| `describe $SVC.Method` | 请求/响应类型签名 |

```bash
grpcurl -plaintext $PROTO_OPTS $BRIDGE list
grpcurl -plaintext $PROTO_OPTS $BRIDGE list $SVC
grpcurl -plaintext $PROTO_OPTS $BRIDGE describe $SVC.GetCapabilities
```

## 1.3 快速冒烟

```bash
# System：能力矩阵
grpcurl -plaintext $PROTO_OPTS -d '{}' "$BRIDGE" "$SVC/GetCapabilities"

# Navigation（另指定 navigation.proto）
export PROTO_OPTS="-import-path $REPO -proto automsgs/proto/rpcs/navigation.proto"
export SVC=automsgs.rpcs.navigation.NavigationService
grpcurl -plaintext $PROTO_OPTS -d "{\"goal_id\":\"$(new_goal_id)\"}" \
  "$BRIDGE" "$SVC/GetStatus"
```

更多场景与预期见 [13 集成测试参考](13_integration_tests.md)。下游业务对照见 [02 §2.5](02_service_overview.md#25-command-与-tasktype-对照)。
