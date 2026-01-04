#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
将 Sphinx 生成的 HTML 文档转换为 PDF

使用方法:
    python html2pdf.py <html_dir> <output_pdf>
    
示例:
    python html2pdf.py build/html Autonomy.pdf
"""

import sys
import os
import argparse
from pathlib import Path

try:
    from playwright.sync_api import sync_playwright
    HAS_PLAYWRIGHT = True
except ImportError:
    HAS_PLAYWRIGHT = False

try:
    import weasyprint
    HAS_WEASYPRINT = True
except ImportError:
    HAS_WEASYPRINT = False


def convert_with_playwright(html_file, output_pdf):
    """使用 Playwright 将 HTML 转换为 PDF"""
    print(f"使用 Playwright 转换 {html_file} 到 {output_pdf}...")
    
    with sync_playwright() as p:
        browser = p.chromium.launch()
        page = browser.new_page()
        
        # 加载 HTML 文件
        html_path = Path(html_file).absolute().as_uri()
        page.goto(html_path, wait_until='networkidle')
        
        # 等待 MathJax 渲染完成（如果有）
        try:
            page.wait_for_function('typeof MathJax !== "undefined" && MathJax.typesetPromise', timeout=5000)
        except:
            pass  # MathJax 可能不存在，继续执行
        
        # 生成 PDF
        page.pdf(
            path=output_pdf,
            format='A4',
            print_background=True,
            margin={
                'top': '1cm',
                'right': '1.5cm',
                'bottom': '1cm',
                'left': '1.5cm'
            }
        )
        
        browser.close()
    
    print(f"✓ PDF 已生成: {output_pdf}")


def convert_with_weasyprint(html_file, output_pdf):
    """使用 WeasyPrint 将 HTML 转换为 PDF"""
    print(f"使用 WeasyPrint 转换 {html_file} 到 {output_pdf}...")
    
    html = weasyprint.HTML(filename=html_file)
    html.write_pdf(output_pdf)
    
    print(f"✓ PDF 已生成: {output_pdf}")


def find_main_html(html_dir):
    """在 HTML 目录中查找主 index.html 文件"""
    html_dir = Path(html_dir)
    index_file = html_dir / 'index.html'
    
    if index_file.exists():
        return str(index_file)
    
    # 查找其他可能的入口文件
    for html_file in html_dir.glob('*.html'):
        if html_file.name in ['index.html', 'contents.html', 'autonomy.html']:
            return str(html_file)
    
    raise FileNotFoundError(f"在 {html_dir} 中未找到 HTML 入口文件")


def main():
    parser = argparse.ArgumentParser(description='将 Sphinx HTML 文档转换为 PDF')
    parser.add_argument('html_dir', help='HTML 文档目录')
    parser.add_argument('output_pdf', nargs='?', default='Autonomy.pdf',
                       help='输出 PDF 文件名（默认: Autonomy.pdf）')
    parser.add_argument('--method', choices=['auto', 'playwright', 'weasyprint'],
                       default='auto', help='转换方法（默认: auto）')
    
    args = parser.parse_args()
    
    # 检查 HTML 目录
    if not os.path.isdir(args.html_dir):
        print(f"错误: {args.html_dir} 不是一个目录")
        return 1
    
    # 查找主 HTML 文件
    try:
        html_file = find_main_html(args.html_dir)
    except FileNotFoundError as e:
        print(f"错误: {e}")
        return 1
    
    print(f"找到 HTML 文件: {html_file}")
    
    # 选择转换方法
    method = args.method
    if method == 'auto':
        if HAS_PLAYWRIGHT:
            method = 'playwright'
        elif HAS_WEASYPRINT:
            method = 'weasyprint'
        else:
            print("错误: 未找到可用的 PDF 转换工具")
            print("请安装以下任一工具:")
            print("  pip install playwright  # 推荐，需要运行: playwright install chromium")
            print("  pip install weasyprint")
            return 1
    
    # 执行转换
    try:
        if method == 'playwright':
            if not HAS_PLAYWRIGHT:
                print("错误: Playwright 未安装")
                print("安装方法: pip install playwright && playwright install chromium")
                return 1
            convert_with_playwright(html_file, args.output_pdf)
        elif method == 'weasyprint':
            if not HAS_WEASYPRINT:
                print("错误: WeasyPrint 未安装")
                print("安装方法: pip install weasyprint")
                return 1
            convert_with_weasyprint(html_file, args.output_pdf)
    except Exception as e:
        print(f"错误: 转换失败 - {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
