# 2. 快速开始

```bash
python3 -m install_deps
python3 tools/python/clang_format_sources.py --check

cmake -G Ninja -B build && ninja -C build
export PATH="$PWD/build/bin:$PATH"
export AUTOLINK_LAUNCH_PATH="$PWD/autonomy/system/launch"
export AUTONOMY_BT_PLUGIN_PATH="$PWD/build/lib"
autolink_launch autonomy.launch

python3 docker/run_autonomy.py -p x86_64 # 可选
```

Bazel（可选）：`./autonomy.sh build -m planning,control`（先有 CMake 前缀）。
