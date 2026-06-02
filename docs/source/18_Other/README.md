# 代码格式化工具

本项目使用 `clang-format` 来统一代码风格，配置文件为项目根目录下的 `.clang-format`。

## 安装 clang-format

在使用格式化工具之前，需要确保已安装 `clang-format`：

### macOS

```bash
brew install clang-format
```

### Linux

```bash
# Ubuntu/Debian
sudo apt-get install clang-format

# CentOS/RHEL
sudo yum install clang-tools-extra
```

## 使用格式化脚本

项目提供了 `scripts/format.py` 脚本来批量格式化代码。

### 基本用法

```bash
# 格式化所有文件（默认格式化 autonomy 和 autolink 目录）
python3 scripts/format.py
```

### 预览模式

在格式化之前，可以使用 `--dry-run` 选项预览将要格式化的文件：

```bash
# 列出将要格式化的所有文件（不实际格式化）
python3 scripts/format.py --dry-run
```

### 检查模式

使用 `--check` 选项可以检查代码格式是否符合规范，而不修改文件：

```bash
# 检查代码格式是否符合规范
python3 scripts/format.py --check
```

如果所有文件都已正确格式化，命令将返回 0；如果有文件需要格式化，将返回非零值。

### 格式化特定目录

可以指定要格式化的目录：

```bash
# 只格式化 autonomy 目录
python3 scripts/format.py autonomy

# 只格式化 autolink 目录
python3 scripts/format.py autolink

# 格式化多个指定目录
python3 scripts/format.py autonomy autolink

# 格式化特定子目录
python3 scripts/format.py autonomy/navigator
```

### 查看帮助

```bash
python3 scripts/format.py --help
```

## 代码风格配置

代码风格配置文件 `.clang-format` 位于项目根目录，主要特点：

- **基于 Google 风格**（BasedOnStyle: Google）
- **缩进宽度**：4 个空格（IndentWidth: 4）
- **不使用 Tab**（UseTab: Never）
- **列限制**：80 字符（ColumnLimit: 80）
- **指针对齐**：左对齐（PointerAlignment: Left）
- **自动排序 include**（SortIncludes: true）
- **自动修复命名空间注释**（FixNamespaceComments: true）

完整的配置选项请参考项目根目录下的 `.clang-format` 文件。

## 注意事项

1. **格式化前建议先提交代码**：格式化会修改文件内容，建议在使用格式化工具之前先提交或备份代码。

2. **批量格式化耗时较长**：当文件数量较多时（如本项目有 1750+ 个文件），格式化可能需要较长时间。

3. **CI/CD 集成**：可以在 CI/CD 流程中使用 `--check` 选项来验证代码格式：

   ```bash
   python3 scripts/format.py --check
   ```

   如果检查失败，构建可以报错并要求开发者格式化代码。

4. **clang-format 版本**：脚本会自动查找以下版本的 clang-format：
   - `clang-format`
   - `clang-format-17`
   - `clang-format-16`
   - `clang-format-15`
   - `clang-format-14`
   - `clang-format-13`

   在 macOS 上，也会自动使用 Apple Command Line Tools 提供的 `clang-format`。

## 示例

```bash
# 1. 预览将要格式化的文件
python3 scripts/format.py --dry-run

# 2. 格式化所有代码
python3 scripts/format.py

# 3. 格式化后检查格式是否正确
python3 scripts/format.py --check

# 4. 只格式化特定模块
python3 scripts/format.py autonomy/navigator
```
