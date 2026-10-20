# 6. Autolink 问题

常见症状：Python 找不到 cyber/protobuf、tcmalloc 崩溃、channel 无数据、record 播放为空。

Autolink 管 **DAG/launch/channel**；序列化用 commsgs；对外 Command 走 **gRPC Bridge**（另一层）。CLI 工具默认 **`AUTOLINK_BUILD_TOOLS=OFF`**。

## 问答

### Q: 找不到 Autolink / cyber Python？

```bash
source /usr/local/setup.bash
```

容器内注意 `PYTHONPATH`（见 [18 · 数据集](../18_Tools/08_datasets.md)）。

### Q: `No module named 'google'` / protobuf 报错？

```bash
python3 -m pip install protobuf==3.14.0 --break-system-packages
```

冲突时用 venv 专跑工具脚本。

### Q: tcmalloc static TLS 警告/崩溃？

```bash
export LD_PRELOAD=/usr/local/lib/libtcmalloc.so.4
```

### Q: autolink_channel 等 CLI 不存在？

嵌入构建默认 **`AUTOLINK_BUILD_TOOLS=OFF`**。需要时在 autolink CMake 开 `-DAUTOLINK_BUILD_TOOLS=ON`，见 [18 · §6](../18_Tools/06_autolink_tools.md)。

### Q: channel 无数据？

发布方 `Init()`、话题名一致（含 `/`）、`autolink_channel list` 看类型。

### Q: record play 空？

record 路径、channel 名与录制一致；bag_convert 见 [18 · §4](../18_Tools/04_bag_convert.md)。
