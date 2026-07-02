# 8. 其他综述

### 8.1 本章内容地图

| 文件 | 适用读者 |
|------|----------|
| [02_code_style.md](02_code_style.md) | 所有开发者 |
| [03_contributing.md](03_contributing.md) | 贡献者 |
| [04_glossary.md](04_glossary.md) | 新手 |
| [05_changelog.md](05_changelog.md) | 维护者 |
| [06_resources.md](06_resources.md) | 学习者 |
| [07_links.md](07_links.md) | 全员 |

### 8.2 文档维护约定

- 技术模块：§1–§9 + `index.rst`
- 工具/杂项：§1–§8 + `index.rst`
- 状态标记：✅ 已实现 / ⏳ 进行中 / ❌ 未开始

### 8.3 本地构建文档

```bash
cd docs
pip install -r requirements.txt
python3 -m sphinx -b html source build
open build/index.html
```

### 8.4 反馈

文档问题请提交 GitHub Issue，标签建议 `documentation`。
