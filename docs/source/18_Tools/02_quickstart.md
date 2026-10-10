# 2. 快速开始

### 2.1 安装依赖

```bash
python3 -m install_deps
```

### 2.2 格式化代码

```bash
# 预览
python3 tools/python/clang_format_sources.py --dry-run

# 格式化
python3 tools/python/clang_format_sources.py

# CI 检查
python3 tools/python/clang_format_sources.py --check
```

### 2.3 端到端验证（多进程）

```bash
cmake -G Ninja -B build && ninja -C build
export PATH="$PWD/build/bin:$PATH"
export AUTOLINK_LAUNCH_PATH="$PWD/autonomy/system/launch"
export AUTONOMY_BT_PLUGIN_PATH="$PWD/build/lib"
autolink_launch autonomy.launch
```

发令请用 Bridge 或 autolink Action Client。详见 [04 Running · 快速运行](../04_Running/02_quickstart.md)。

### 2.4 Docker 开发环境

```bash
python3 docker/run_autonomy.py -p x86_64
docker exec -it SpaceHero /bin/bash
```

### 2.5 Bazel 域库（可选，Bzlmod）

```bash
# 先有 CMake 前缀（automsgs / autolink），再编域库
export AUTONOMY_PREFIX="$PWD/build"   # 或 install 前缀

./autonomy.sh deps                    # BCR + prefix 一览
./autonomy.sh build -m planning,control
# 等价: bazel build //autonomy/planning:autonomy_planning //autonomy/planning:autonomy.planning
```

依赖：`MODULE.bazel`（BCR）+ `@autonomy_prefix`（CMake）。域图与宏：`tools/package.bzl`。  
改依赖后刷新锁：`bazel mod deps --lockfile_mode=update`（提交 `MODULE.bazel.lock`）。

### 2.6 下一步

| 目标 | 文档 |
|------|------|
| 脚本详解 | [§3 开发脚本](03_dev_scripts.md) |
| 多进程栈 | [04 Running · 多进程栈](../04_Running/03_autonomy_process.md) |
| Autolink 调试 | [§5 Autolink 工具](06_autolink_tools.md) |
