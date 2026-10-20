# 6. Autolink 工具

嵌入构建默认 `AUTOLINK_BUILD_TOOLS=OFF`。独立构建：`-DAUTOLINK_BUILD_TOOLS=ON`。

| 工具 | 用途 |
|------|------|
| `autolink_channel` / `node` / `service` / `action` | 列表与调试 |
| `autolink_monitor` | 拓扑 |
| `autolink_recorder` | 录制回放 |
| `autolink_launch` | launch |
| `autolink_mainboard` | 模块运行时 |

```bash
autolink_channel list
autolink_node list
autolink_recorder record -o /tmp/autolink.record
```
