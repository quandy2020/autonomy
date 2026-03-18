# HTML 转 PDF 文档生成指南

本文档说明如何将 Sphinx 生成的 HTML 文档转换为 PDF 格式。

## 方法概述

提供了两种转换方法：

1. **Playwright**（推荐）- 基于 Chromium，支持完整 JavaScript 和 CSS 渲染
2. **WeasyPrint** - 纯 Python 实现，CSS 支持良好

## 安装依赖

### 方法 1: 使用 Playwright（推荐）

```bash
pip install playwright
playwright install chromium
```

**优点**：
- 完整的浏览器渲染，支持所有 JavaScript 和 CSS
- 完美支持 MathJax 数学公式
- 支持复杂布局和交互元素

**缺点**：
- 需要下载 Chromium（约 150MB）
- 内存占用较高

### 方法 2: 使用 WeasyPrint

```bash
pip install weasyprint
```

**优点**：
- 纯 Python 实现，无需额外浏览器
- 安装简单
- 内存占用低

**缺点**：
- 不支持 JavaScript（MathJax 可能无法渲染）
- CSS 支持有限

## 使用方法

### 方法 1: 使用提供的 Python 脚本

```bash
cd src/autonomy/docs

# 首先生成 HTML 文档
sphinx-build -b html source build/html

# 转换为 PDF（自动选择可用工具）
python html2pdf.py build/html Autonomy.pdf

# 指定使用 Playwright
python html2pdf.py build/html Autonomy.pdf --method playwright

# 指定使用 WeasyPrint
python html2pdf.py build/html Autonomy.pdf --method weasyprint
```

### 方法 2: 使用 CMake 构建目标

```bash
cd build
cmake ..
make build_doc      # 生成 HTML
make build_doc_pdf  # 转换为 PDF
```

生成的 PDF 文件位于：`build/docs/Autonomy.pdf`

### 方法 3: 手动使用浏览器

如果你有 Chrome/Chromium 浏览器，也可以手动转换：

1. 在浏览器中打开 `build/html/index.html`
2. 使用浏览器的打印功能（Ctrl+P 或 Cmd+P）
3. 选择"另存为 PDF"
4. 设置页面边距和选项
5. 保存为 PDF

## 脚本选项

```bash
python html2pdf.py <html_dir> [output_pdf] [--method METHOD]

参数:
  html_dir      HTML 文档目录（例如: build/html）
  output_pdf    输出 PDF 文件名（默认: Autonomy.pdf）
  --method      转换方法: auto, playwright, weasyprint
```

## 故障排除

### 问题 1: Playwright 浏览器未安装

**错误信息**：
```
Error: Executable doesn't exist
```

**解决方法**：
```bash
playwright install chromium
```

### 问题 2: WeasyPrint 依赖缺失

**错误信息**：
```
OSError: no library called "cairo" was found
```

**解决方法**：

**Ubuntu/Debian**:
```bash
sudo apt-get install python3-cffi python3-brotli libpango-1.0-0 libpangoft2-1.0-0
```

**macOS**:
```bash
brew install cairo pango gdk-pixbuf libffi
```

**Windows**:
安装 GTK+ 运行时或使用 Playwright 方法。

### 问题 3: MathJax 公式未渲染

**原因**：WeasyPrint 不支持 JavaScript，无法渲染 MathJax。

**解决方法**：
- 使用 Playwright 方法（支持 JavaScript）
- 或者在 Sphinx 配置中使用 `sphinx.ext.imgmath` 将数学公式转换为图片

### 问题 4: PDF 格式不正确

**解决方法**：
- 检查 HTML 文档是否正常生成
- 尝试使用不同的转换方法
- 检查 CSS 样式是否兼容（WeasyPrint 对某些 CSS 特性支持有限）

## 性能优化

### Playwright 优化

- 设置 `wait_until='networkidle'` 确保页面完全加载
- 等待 MathJax 渲染完成（脚本已包含）
- 调整页面边距以节省空间

### WeasyPrint 优化

- 简化复杂的 CSS（避免使用不支持的特性）
- 使用相对路径引用资源
- 预先处理图片大小

## 输出质量对比

| 特性 | Playwright | WeasyPrint |
|------|-----------|------------|
| JavaScript 支持 | ✅ 完整支持 | ❌ 不支持 |
| CSS 支持 | ✅ 完整支持 | ⚠️ 部分支持 |
| MathJax 渲染 | ✅ 支持 | ❌ 不支持 |
| 性能 | 中等 | 快 |
| 内存占用 | 高 | 低 |
| 安装复杂度 | 中等 | 低 |

## 建议

- **推荐使用 Playwright**：对于包含数学公式和复杂样式的文档
- **使用 WeasyPrint**：对于简单文档或 CI/CD 环境（无需浏览器）
- **手动转换**：对于一次性转换或需要精细控制的情况

## 注意事项

1. **首次使用 Playwright**：需要下载 Chromium，可能需要几分钟
2. **大文档**：转换大文档可能需要较长时间和更多内存
3. **图片路径**：确保图片使用相对路径，否则 PDF 中可能无法显示
4. **字体**：确保系统安装了文档中使用的字体
