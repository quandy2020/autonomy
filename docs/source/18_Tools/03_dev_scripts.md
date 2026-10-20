# 3. 开发工具

### 3.1 格式化

```bash
python3 tools/python/clang_format_sources.py # 写
python3 tools/python/clang_format_sources.py --check # CI
python3 tools/python/clang_format_sources.py --dry-run
```

依据根目录 `.clang-format`。

### 3.2 依赖

```bash
python3 -m install_deps
python3 -m install_deps --apt-only
python3 -m install_deps --thirdparty-only
python3 -m install_deps --resume-from install_opencv.sh
```

面向 Ubuntu；第三方脚本在 `docker/install/`。
