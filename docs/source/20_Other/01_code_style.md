# 1. 代码风格

改 `autonomy/`、`autolink/` 下 C++ 时需符合根目录 `.clang-format`（Google 基线、4 空格、80 列）；CI 可能只跑 `--check`。

提 PR 前对 touched 文件跑 format；动 Bridge 时用 **`BUILD_GRPC=ON`** 本地编过再交。

### 格式化

```bash
sudo apt-get install clang-format   # 或 brew install clang-format
python3 tools/python/clang_format_sources.py          # 写回
python3 tools/python/clang_format_sources.py --check  # 只检查
python3 tools/python/clang_format_sources.py --dry-run
```

只改文档可不跑；改 C++ 建议全量或至少 touched 文件。

### 命名习惯

跟相邻文件走；Bridge Handler 用现有 `BRIDGE_*` 宏；proto 字段 snake_case，包名在 `automsgs` 下保持一致。
