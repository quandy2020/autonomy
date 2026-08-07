# Autoviz Python Examples

Small tutorials that publish [`automsgs`](../../automsgs) messages over
[`autolink`](../../autolink) for live Autoviz verification.

## Prerequisites

1. Build the autonomy workspace (`colcon build` — includes `automsgs` and
   **autolink Python** by default).

   Python module path: `build/autonomy/python/`

   If you configured before this default, re-run once:

```bash
colcon build --cmake-args -DAUTOLINK_BUILD_PYTHON=ON
```

   Fallback standalone build:

```bash
./src/autonomy/autoviz/examples/python/setup_autolink_python.sh
```

2. Install protobuf if needed:

```bash
/usr/bin/python3 -m pip install protobuf
```

**Isaac Sim / Docker note:** `.bashrc` often aliases `python3` to Isaac Sim's
`python.sh`, which causes `SRE module mismatch`. Either:

```bash
./examples/python/01_tutorial_show_path.sh
# or
/usr/bin/python3 examples/python/01_tutorial_show_path.py
```

Bare `python3 ...` is also handled via auto re-exec when possible.

Optional overrides:

```bash
export AUTONOMY_BUILD_DIR=/path/to/build/autonomy
export AUTOLINK_PYTHON_DIR=/path/to/build/autolink-python
export AUTOVIZ_PYTHON=/opt/venv/bin/python3
```

## 01 — Show Path

Publishes `automsgs.msgs.nav_msgs.Path` on `/fake/path` (default Autoviz
`config/default.autoviz` channel). Fixed Frame should be `map`.

Terminal 1 — publish:

```bash
cd src/autonomy/autoviz
/usr/bin/python3 examples/python/01_tutorial_show_path.py
```

Terminal 2 — echo messages (verify data):

```bash
./build/autonomy/bin/autolink channel echo /fake/path --once
# continuous: ./build/autonomy/bin/autolink channel echo /fake/path
# first 5:   ./build/autonomy/bin/autolink channel echo /fake/path -n 5
```

Terminal 3 — visualize (optional):

```bash
./build/autonomy/bin/autoviz -c config/default.autoviz
```

**Path not visible in 3D?**

1. Fixed Frame 必须是 `map`（与教程 `--frame map` 一致）
2. Path Display 需勾选 Enabled，Channel 为 `/fake/path`
3. 先启动 publisher，再启动 autoviz；或在 Channels 面板确认 `/fake/path` 有消息计数
4. 用 echo 验证同环境能收到数据：
   `./build/autonomy/bin/autolink channel echo /fake/path --once`
5. publisher 与 autoviz 必须在同一 Docker 容器/主机（DDS stub 构建不支持跨容器 RTPS）

Options:

```bash
python3 examples/python/01_tutorial_show_path.py --static
python3 examples/python/01_tutorial_show_path.py --channel /demo/path --frame map
```
