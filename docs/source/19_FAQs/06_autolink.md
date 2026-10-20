# 6. Autolink 问题

### Q: 找不到 Autolink / cyber Python 模块？

```bash
source /usr/local/setup.bash
```

### Q: `No module named 'google'`？

```bash
python3 -m pip install protobuf==3.14.0 --break-system-packages
# 或: sudo apt install python3-protobuf
```

### Q: tcmalloc static TLS？

```bash
export LD_PRELOAD=/usr/local/lib/libtcmalloc.so.4
```

### Q: CLI 找不到？

嵌入构建默认关工具；`-DAUTOLINK_BUILD_TOOLS=ON`，见 [18 · §6](../18_Tools/06_autolink_tools.md)。

### Q: 通道无数据？

确认 `Init()`、话题名一致；`autolink_channel list`。
