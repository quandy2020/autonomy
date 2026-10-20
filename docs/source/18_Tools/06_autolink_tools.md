# 6. Autolink 工具

独立构建 autolink 时：`-DAUTOLINK_BUILD_TOOLS=ON`。
| 工具 | 用途 |
|------|------|
| `autolink_channel` | 列 channel / 类型 |
| `autolink_node` | 节点列表 |
| `autolink_service` / `action` | 服务与 action 调试 |
| `autolink_monitor` | 拓扑 |
| `autolink_recorder` | record / play |
| `autolink_launch` | 起 launch（`autonomy.launch`） |
| `autolink_mainboard` | 单模块 DAG |
```bash
autolink_channel list
autolink_node list
autolink_recorder record -o /tmp/autolink.record
```

**什么时候用：** 查 channel、录包、起 launch；**嵌入 autonomy 构建默认不编全量 CLI**。

### 常见坑

| 现象 | 说明 |
|------|------|
| command not found | 未 `export PATH=build/bin` 或未开 BUILD_TOOLS |
| list 空 | 栈未起或未 Init |
| play 无数据 | 话题名 leading `/` 与 record 不一致 |

与 FAQ：[06 Autolink](../19_FAQs/06_autolink.md)。
