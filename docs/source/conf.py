# Configuration file for the Sphinx documentation builder.

# -- Project information -----------------------------------------------------

project = "Autonomy"
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
    'myst_parser',
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
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]

# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = "sphinx_rtd_theme"

# 主题选项 - 改善视觉效果
html_theme_options = {
    'logo_only': False,
    'display_version': True,
    'prev_next_buttons_location': 'bottom',
    'style_external_links': True,
    'vcs_pageview_mode': '',
    'style_nav_header_background': 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
    # 侧边栏选项
    'collapse_navigation': True,
    'sticky_navigation': True,
    'navigation_depth': 4,
    'includehidden': True,
    'titles_only': False
}

# Add custom CSS to increase page width and improve appearance
# CSS file is in source root directory
html_static_path = ["."]
html_css_files = ["custom.css"]

# 代码高亮主题 - 使用 monokai 暗色主题（与 CSS 代码块暗色背景匹配）
# 可选主题: default, tango, friendly, colorful, autumn, murphy, manni, monokai, perldoc, pastie, borland, trac, native, fruity, bw, emacs, vim, vs, rrt, xcode, igor, paraiso-light, paraiso-dark, lovelace, algol, algol_nu, arduino, rainbow_dash, abap, solarized-dark, solarized-light, sas, stata, stata-light, stata-dark, inkpot
pygments_style = 'monokai'

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

# -- Copy button configuration for code blocks --------------------------------
# 代码块复制按钮配置
copybutton_prompt_text = r">>> |\.\.\. |\$ |In \[\d+\]: | {2,5}\.\.\.: | {5,8}: "
copybutton_prompt_is_regexp = True
copybutton_only_copy_prompt_lines = True
copybutton_remove_prompts = True
