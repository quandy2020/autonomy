# 3. 贡献指南

### 3.1 开始之前

1. Fork 仓库：[quandy2020/autonomy](https://github.com/quandy2020/autonomy)
2. 阅读 [01 Instructions](../01_Instructions/index.rst) 了解架构
3. 按 [02 Installation](../02_Installation/index.rst) 搭建环境

### 3.2 开发流程

```bash
git checkout -b feature/your-topic
# 修改代码
python3 scripts/format.py
cmake -G Ninja -B build && ninja -C build
# 运行相关测试
git commit -m "feat: your change"
git push origin feature/your-topic
```

### 3.3 提交规范

建议使用语义化前缀：

| 前缀 | 用途 |
|------|------|
| `feat:` | 新功能 |
| `fix:` | 缺陷修复 |
| `docs:` | 文档 |
| `refactor:` | 重构 |
| `test:` | 测试 |
| `chore:` | 构建/工具 |

### 3.4 代码要求

- C++17，遵循 `.clang-format`
- 新模块补充 `index.rst` + §1–§8 文档（若适用）
- 避免引入未使用的重量级依赖
- 与 Navigation2 对齐的接口保持命名一致

### 3.5 Pull Request

1. 描述变更动机与测试方式
2. 关联相关 Issue（若有）
3. 确保 `format.py --check` 通过
4. 文档与代码同步更新

### 3.6 许可证

贡献代码即表示同意 [Apache License 2.0](https://github.com/quandy2020/autonomy/blob/main/LICENSE)。
