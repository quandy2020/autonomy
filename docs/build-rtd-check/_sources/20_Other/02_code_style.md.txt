# 2. 代码风格

本项目使用 **`clang-format`** 统一 C/C++ 代码风格，配置文件为根目录 **`.clang-format`**。

### 2.1 安装 clang-format

**macOS**

```bash
brew install clang-format
```

**Linux（Ubuntu/Debian）**

```bash
sudo apt-get install clang-format
```

**CentOS/RHEL**

```bash
sudo yum install clang-tools-extra
```

### 2.2 使用格式化脚本

```bash
# 格式化全部
python3 scripts/format.py

# 预览
python3 scripts/format.py --dry-run

# CI 检查
python3 scripts/format.py --check

# 指定目录
python3 scripts/format.py autonomy/navigator
```

详见 [18 Tools · 开发脚本](../18_Tools/03_dev_scripts.md)。

### 2.3 `.clang-format` 要点

| 选项 | 值 |
|------|-----|
| 基础风格 | Google（`BasedOnStyle: Google`） |
| 缩进 | 4 空格（`IndentWidth: 4`） |
| Tab | 不使用（`UseTab: Never`） |
| 列宽 | 80（`ColumnLimit: 80`） |
| 指针对齐 | 左对齐（`PointerAlignment: Left`） |
| include 排序 | 开启（`SortIncludes: true`） |
| 命名空间注释 | 自动修复（`FixNamespaceComments: true`） |

完整选项见项目根目录 `.clang-format`。

### 2.4 注意事项

1. 格式化会修改文件，建议先提交或备份
2. 全量格式化 1700+ 文件可能耗时较长
3. CI 可使用 `python3 scripts/format.py --check` 门禁
4. 脚本自动查找 `clang-format-13` … `clang-format-17`

### 2.5 示例工作流

```bash
python3 scripts/format.py --dry-run
python3 scripts/format.py
python3 scripts/format.py --check
```
