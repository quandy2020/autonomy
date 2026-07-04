# Configuration file for the Sphinx documentation builder.

import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

# section_heading_slug registered via import path in myst_heading_slug_func

# -- Project information -----------------------------------------------------

project = "Autonomy"
html_title = "Autonomy 文档"
html_short_title = "Autonomy"
copyright = "2026 OpenRobotics Team"
author = "OpenRobotics Team"


# -- General configuration ---------------------------------------------------
# -- General configuration

extensions = [
    "sphinx.ext.duration",
    "sphinx.ext.doctest",
    "sphinx.ext.autodoc",
    "sphinx.ext.autosummary",
    "sphinx.ext.intersphinx",
    "sphinx.ext.mathjax",  # 支持数学公式
    "sphinx_copybutton",  # 代码块一键复制功能
    "myst_parser",
    "sphinxcontrib.mermaid",  # Mermaid 图表渲染
]

intersphinx_mapping = {
    "rtd": ("https://docs.readthedocs.io/en/stable/", None),
    "python": ("https://docs.python.org/3/", None),
    "sphinx": ("https://www.sphinx-doc.org/en/master/", None),
}
intersphinx_disabled_domains = ["std"]

source_suffix = {
    '.rst': 'restructuredtext',
    '.txt': 'markdown',
    '.md': 'markdown',
}

templates_path = ["_templates"]

# -- Options for EPUB output
epub_show_urls = "footnote"

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store", "**/guide.md"]

# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = "sphinx_rtd_theme"

# 主题选项 — 简约清晰风格
html_theme_options = {
    'logo_only': False,
    'prev_next_buttons_location': 'bottom',
    'style_external_links': True,
    'vcs_pageview_mode': '',
    'style_nav_header_background': '#7aa88d',
    'collapse_navigation': True,
    'sticky_navigation': True,
    'navigation_depth': 4,
    'includehidden': True,
    'titles_only': False,
}

# Add custom CSS to increase page width and improve appearance
# CSS file is in source root directory
html_static_path = ["."]
html_css_files = ["custom.css"]

# 代码高亮 — 浅色主题，与正文风格一致
pygments_style = 'friendly'

# -- MathJax configuration for math formulas ---------------------------------
# MathJax 配置 - 支持 LaTeX 数学公式
# 使用 MathJax 3.x CDN
mathjax_path = "https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-mml-chtml.js"

# 配置 myst_parser 以支持数学公式
myst_enable_extensions = [
    "amsmath",
    "dollarmath",  # 支持 $...$ 和 $$...$$ 数学公式
    "colon_fence",  # 支持 ::: 代码块
]
myst_math_delimiters = [
    ("$$", "$$"),
    ("\\[", "\\]"),
]

# 将 Markdown 中的 ```mermaid 代码块映射为 sphinxcontrib-mermaid 指令
# 与 GitHub / 编辑器预览语法保持一致，无需改为 ```{mermaid}
myst_fence_as_directive = ["mermaid", "toctree"]

# 为标题生成锚点（含 #### 小节），与文档内 #410-xxx 链接格式一致
myst_heading_anchors = 6
myst_heading_slug_func = "sphinx_ext.heading_slugs.section_heading_slug"

# §N 章节使用「# 1. 标题」+「### N.M 小节」；跳过 H1→H3 跳级告警
suppress_warnings = ["myst.header"]

# -- Mermaid configuration -----------------------------------------------------
# 使用 jsDelivr CDN 加载 Mermaid（支持 timeline、flowchart 等）
mermaid_version = "10.9.1"

# 全局初始化：禁用 useMaxWidth 避免图表被压缩过小，统一字号
mermaid_init_js = """
mermaid.initialize({
  startOnLoad: false,
  theme: 'base',
  themeVariables: {
    fontSize: '14px',
    fontFamily: 'system-ui, -apple-system, "Segoe UI", sans-serif',
    primaryColor: '#e0f2f2',
    primaryTextColor: '#1a4d6e',
    primaryBorderColor: '#2d9294',
    lineColor: '#5a7289',
    secondaryColor: '#eef1f4',
    tertiaryColor: '#ffffff'
  },
  flowchart: {
    useMaxWidth: false,
    htmlLabels: true,
    curve: 'basis',
    padding: 20,
    nodeSpacing: 45,
    rankSpacing: 55
  },
  timeline: {
    disableMulticolor: false
  }
});
"""

# -- Copy button configuration for code blocks --------------------------------
# 代码块复制按钮配置
copybutton_prompt_text = r">>> |\.\.\. |\$ |In \[\d+\]: | {2,5}\.\.\.: | {5,8}: "
copybutton_prompt_is_regexp = True
copybutton_only_copy_prompt_lines = True
copybutton_remove_prompts = True

# -- Local extensions / HTML language -----------------------------------------
html_language = "zh-CN"


def setup(app):
    """Register local Sphinx helpers (heading HTML ids, etc.)."""
    from sphinx_ext.heading_slugs import register_heading_id_transform

    register_heading_id_transform(app)
    return {"version": "1", "parallel_read_safe": True, "parallel_write_safe": True}
