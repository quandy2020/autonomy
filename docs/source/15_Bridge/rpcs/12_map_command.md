(rpc-map-command)=
# MapService

源文件：`automsgs/proto/rpcs/mapping.proto`  
Handler：`rpc_map_handlers` → `MapServiceStub` / `MappingStub`

建图会话 + 地图资源管理（均为 Unary，无旧版 `SendMapCommand` 流）。

## 12.1 方法

```protobuf
service MapService {
  rpc StartMapping(StartMappingRequest) returns (automsgs.rpcs.common.Status);
  rpc FinishMapping(FinishMappingRequest) returns (FinishMappingResponse);
  rpc CancelMapping(CancelMappingRequest) returns (automsgs.rpcs.common.Status);
  rpc GetMappingStatus(GetMappingStatusRequest) returns (MappingStatus);

  rpc ListMaps(ListMapsRequest) returns (ListMapsResponse);
  rpc GetMap(GetMapRequest) returns (GetMapResponse);
  rpc GetMapMetadata(GetMapMetadataRequest) returns (GetMapMetadataResponse);
  rpc SaveMap(SaveMapRequest) returns (SaveMapResponse);
  rpc DeleteMap(DeleteMapRequest) returns (automsgs.rpcs.common.Status);
  rpc SetCurrentMap(SetCurrentMapRequest) returns (automsgs.rpcs.common.Status);
}
```

| 分组 | RPC | 说明 |
|------|-----|------|
| 会话 | `StartMapping` / `FinishMapping` / `CancelMapping` / `GetMappingStatus` | SLAM 生命周期 |
| 存储 | `ListMaps` / `GetMap` / `GetMapMetadata` / `SaveMap` / `DeleteMap` / `SetCurrentMap` | 地图资源 |

## 12.2 要点

- 优先 `map_identifier`；`map_name` 在 identifier 空时作别名查找。
- 忙 → `MAPPING_BUSY`（1100）。

```bash
export PROTO_OPTS="-import-path $REPO -proto automsgs/proto/rpcs/mapping.proto"
export SVC=automsgs.rpcs.mapping.MapService
grpcurl -plaintext $PROTO_OPTS -d '{}' "$BRIDGE" "$SVC/ListMaps"
grpcurl -plaintext $PROTO_OPTS -d '{}' "$BRIDGE" "$SVC/GetMappingStatus"
```
