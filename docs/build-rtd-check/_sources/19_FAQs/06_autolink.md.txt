# 6. Autolink 问题

> 历史文档中常见 **Cyber RT** 相关问题。Autonomy 当前使用 **Autolink** 作为通信中间件，排错思路类似。

### Q: `No module named 'cyber'` 或找不到 Autolink Python 模块？

确保已 source 安装路径下的环境脚本：

```bash
source /usr/local/setup.bash
# 或项目 build 目录下的 setup 脚本（若已安装）
```

若使用独立 Autolink 构建，参考 `autolink/README.md` 中的 Python 绑定说明。

### Q: `No module named 'google'`（protobuf）？

```text
ModuleNotFoundError: No module named 'google'
```

安装 Protobuf Python 包：

```bash
python3 -m pip install protobuf==3.14.0 --break-system-packages
# Ubuntu 亦可：sudo apt install python3-protobuf
```

版本需与系统 Protobuf C++ 库兼容。

### Q: `cannot allocate memory in static TLS block`（libtcmalloc）？

```text
ImportError: /usr/local/lib/libtcmalloc.so.4: cannot allocate memory in static TLS block
```

在运行前预加载 tcmalloc：

```bash
export LD_PRELOAD=/usr/local/lib/libtcmalloc.so.4
```

或将上述行加入 `~/.bashrc`（gperftools 与 Python 扩展混用时常见）。

### Q: Autolink CLI 工具找不到？

嵌入 Autonomy 构建时默认 `AUTOLINK_BUILD_TOOLS=OFF`。需独立构建 Autolink 并启用 `-DAUTOLINK_BUILD_TOOLS=ON`，见 [18 Tools · Autolink 工具](../18_Tools/06_autolink_tools.md)。

### Q: 通道无数据 / 节点列表为空？

1. 确认 `autolink::Init()` 已调用
2. 检查 Writer/Reader 话题名是否一致
3. 使用 `autolink_channel list`（若已构建 CLI）

详见 [03 Communication](../03_Communication/index.rst)。
