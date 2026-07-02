# 6. Autolink 工具

Autolink 提供一组命令行工具，用于调试通道、节点、服务、Action 与录制回放。

### 6.1 构建条件

嵌入 Autonomy 构建时，根 `CMakeLists.txt` 通常设置 `AUTOLINK_BUILD_TOOLS=OFF`，**默认不生成**下列 CLI。

独立构建 Autolink 子项目时，可启用：

```bash
cmake -DAUTOLINK_BUILD_TOOLS=ON ...
```

### 6.2 工具列表

| 可执行文件 | 用途 |
|------------|------|
| `autolink_channel` | 通道信息、带宽/频率统计 |
| `autolink_node` | 节点列表与信息 |
| `autolink_service` | 服务列表与信息 |
| `autolink_action` | Action 列表、发送 goal |
| `autolink_monitor` | 终端拓扑/通道监控 |
| `autolink_recorder` | 录制、回放、分割、恢复 |
| `autolink_launch` | XML launch 编排 |
| `autolink_mainboard` | 模块加载运行时 |

源码目录：`autolink/autolink/tools/`

### 6.3 典型用法

```bash
# 列出活跃通道
autolink_channel list

# 查看节点
autolink_node list

# 录制
autolink_recorder record -o /tmp/autolink.record
```

### 6.4 相关文档

- [03 Communication](../03_Communication/index.rst)
- Autolink README：`autolink/README.md`
