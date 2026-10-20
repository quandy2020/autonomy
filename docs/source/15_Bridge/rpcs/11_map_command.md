(rpc-map-command)=
# MapService

源：`mapping.proto`。

| 分组 | RPC |
|------|-----|
| 会话 | Start/Finish/Cancel Mapping · GetMappingStatus |
| 存储 | List/Get/Save/Delete Map · GetMapMetadata · SetCurrentMap |

优先 `map_identifier`。忙 → `MAPPING_BUSY`。

```bash
export PROTO_OPTS="-import-path $REPO -proto automsgs/proto/rpcs/mapping.proto"
export SVC=automsgs.rpcs.mapping.MapService
grpcurl -plaintext $PROTO_OPTS -d '{}' "$BRIDGE" "$SVC/ListMaps"
```
