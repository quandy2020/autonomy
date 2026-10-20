# 0. Other 指南

## 背景

提 PR 前格式不对、文档缩写对不上、想查版本差异却翻错文件——本章放 **规范与外围资料**，不涉及算法或运行步骤。各域「怎么跑」仍在 02–18；这里管 **怎么写、怎么贡献、术语与外链**。

## 思路

- 代码风格：`tools/python/clang_format_sources.py`，根目录 `.clang-format`（Google 基、4 空格、80 列）。  
- 贡献流程以 [02 贡献](02_contributing.md) 为准；改文档也建议跑相关模块基本编译。  
- 完整版本记录在仓库根 **`CHANGELOG.rst`**；[04 Changelog](04_changelog.md) 是文档侧摘要。  
- [03 术语](03_glossary.md) 是速查，细节仍以各域指南为准。


## 步骤

### 1. 各篇什么时候翻

| 文档 | 解决什么 |
|------|----------|
| [01 代码风格](01_code_style.md) | PR 前；clang-format 约定 |
| [02 贡献](02_contributing.md) | 第一次给仓库提改动 |
| [03 术语](03_glossary.md) | 缩写与名词 |
| [04 Changelog](04_changelog.md) | 版本差异入口 |
| [05 资源](05_resources.md) | 外部论文 / 项目 |
| [06 链接](06_links.md) | GitHub、在线文档等 |

### 2. 常见坑

| 现象 | 先查 |
|------|------|
| CI 格式不过 | [01](01_code_style.md)；clang-format 脚本 |
| 术语和代码不一致 | [03](03_glossary.md) + 对应域指南 |
| 只看文档 changelog 不全 | 根目录 `CHANGELOG.rst` |
