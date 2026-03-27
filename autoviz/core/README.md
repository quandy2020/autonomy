# autoviz `core/`（单一数据流）

**一条主链路**：autolink 上的 automsgs（protobuf）→ **Foxglove Studio** 可视化；可选 **MCAP** 落盘。

```
Topology 发现 topic → AutolinkBridge 订阅 → FoxgloveServer（protobuf 通道）
                                              ↘ 有映射时 topic/foxglove（官方 schema）
```

| 目录 | 职责 |
|------|------|
| **`settings.hpp`** | `FoxgloveConfig` / `AutolinkConfig` / `McapConfig` 等 POD；`App::Initialize` 用 `autolink::common::LoadConfig` 读 `config/autoviz.pb.conf`（`proto/autoviz_conf.proto`）后填充 |
| **`foxglove/`** | `FoxgloveServer`：`EmitProtobuf` / `EmitFoxglove` |
| **`autolink/`** | `AutolinkBridge`：`SubscribeSerialized`、`PublishSerialized` → 上述 `Emit*` |
| **`recorder/`** | 可选 MCAP |
| **`convert/`** | automsgs → Foxglove schema（见 `convert/README.md`） |

`App`：`Initialize` → `FoxgloveServer` → `AutolinkBridge`（含发现）→ `Recorder`；退出前 `Shutdown`。

**proto 变更后**：运行 `core/convert/tools/generate_automsgs_foxglove_registry.py`，更新 `registry/automsgs_foxglove_registry_data.hpp`。
