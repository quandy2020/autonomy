(rpc-map-command)=
# MapService

**源文件：** `automsgs/proto/rpcs/mapping.proto`。

在建图会话、切换/保存/删除地图，**不**替代导航 Goal（导航仍走 NavigationService）

- 引用地图优先 `map_identifier`，避免只靠显示名。  
- `SetCurrentMap` 后 planning 用的 occupancy 才会切；车不动时查 map 进程是否加载。  
- Save/Delete 可能较慢，Unary  deadline 要留够。

### RPC 分组

| 分组 | RPC |
|------|-----|
| 建图会话 | `StartMapping` / `FinishMapping` / `CancelMapping` / `GetMappingStatus` |
| 地图库 | `ListMaps` / `GetMap` / `SaveMap` / `DeleteMap` / `GetMapMetadata` / `SetCurrentMap` |

多数为 **Unary**；与 Command Stream 无混用。

```bash
export PROTO_OPTS="-import-path $REPO -proto automsgs/proto/rpcs/mapping.proto"
export SVC=automsgs.rpcs.mapping.MapService
grpcurl -plaintext $PROTO_OPTS -d '{}' "$BRIDGE" "$SVC/ListMaps"
```

### 互斥

`StartMapping` 在已有 MAPPING 会话时 → `MAPPING_BUSY`（Status 帧或 FAILED_PRECONDITION，以实现为准）。  
与导航互斥策略取决于 task 配置；联调时习惯先 Cancel 导航再建图。

### 常见坑

| 现象 | 说明 |
|------|------|
| List 空 | map 服务未起或未导入 |
| Start 立刻 busy | 上次会话未 Finish/Cancel |
| 切图后规划仍旧图 | SetCurrentMap 未成功；或 planner 缓存 |
