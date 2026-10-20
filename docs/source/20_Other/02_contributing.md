# 2. 贡献指南

```bash
git checkout -b feature/topic
python3 tools/python/clang_format_sources.py
cmake -G Ninja -B build && ninja -C build
git commit -m "feat: …"
```

| 前缀 | 用途 |
|------|------|
| `feat:` / `fix:` / `docs:` | 功能 / 修复 / 文档 |
| `refactor:` / `test:` / `chore:` | 重构 / 测试 / 工具 |

要求：C++17、`.clang-format`、PR 含动机与测试；许可 Apache-2.0。
