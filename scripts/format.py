#!/usr/bin/env python3
"""
代码格式化脚本
根据 .clang-format 配置文件格式化 autonomy 和 autolink 目录下的 C/C++ 代码
"""

import argparse
import subprocess
import sys
import os
import shutil
from pathlib import Path


def get_project_root():
    """获取项目根目录"""
    script_dir = Path(__file__).parent.absolute()
    # scripts 目录的父目录是项目根目录
    return script_dir.parent


def find_clang_format():
    """查找 clang-format 工具"""
    # 首先尝试标准 PATH 中的 clang-format（包括版本化名称）
    for name in ("clang-format", "clang-format-17", "clang-format-16", 
                 "clang-format-15", "clang-format-14", "clang-format-13"):
        path = shutil.which(name)
        if path:
            return path
    
    # 尝试通过 xcode-select 获取的路径查找 Apple Command Line Tools
    try:
        xcode_path_result = subprocess.run(
            ["xcode-select", "-p"],
            capture_output=True,
            text=True,
            check=False
        )
        if xcode_path_result.returncode == 0:
            xcode_path = xcode_path_result.stdout.strip()
            # 尝试在 xcode-select 返回的路径下查找 clang-format
            clang_format_path = os.path.join(xcode_path, "usr", "bin", "clang-format")
            if os.path.exists(clang_format_path) and os.access(clang_format_path, os.X_OK):
                return clang_format_path
    except (subprocess.SubprocessError, FileNotFoundError):
        pass
    
    return None


def list_source_files(root_dir, directories):
    """
    列出指定目录中的 C/C++ 源文件
    
    Args:
        root_dir: 项目根目录
        directories: 要搜索的目录列表（相对于 root_dir）
    
    Returns:
        文件路径列表（相对于 root_dir）
    """
    files = []
    root_path = Path(root_dir).resolve()
    
    # C/C++ 文件扩展名
    extensions = (".c", ".cc", ".cpp", ".cxx", ".h", ".hpp", ".hxx", ".h++")
    
    for dir_name in directories:
        # 确保目录名是相对于 root_dir 的
        dir_path = root_path / dir_name
        if not dir_path.exists():
            print(f"Warning: Directory {dir_path} does not exist, skipping...", 
                  file=sys.stderr)
            continue
        
        if not dir_path.is_dir():
            print(f"Warning: {dir_path} is not a directory, skipping...", 
                  file=sys.stderr)
            continue
        
        # 递归查找所有 C/C++ 源文件
        for ext in extensions:
            for file_path in dir_path.rglob(f"*{ext}"):
                if file_path.is_file():
                    # 转换为相对路径（相对于 root_dir）
                    files.append(str(file_path.relative_to(root_path)))
    
    # 去重并排序
    files = sorted(set(files))
    return files


def format_files(directories=None, dry_run=False, check=False):
    """
    格式化项目中的 C/C++ 源文件
    
    Args:
        directories: 要格式化的目录列表，默认为 ["autonomy", "autolink"]
        dry_run: 如果为 True，只列出将要格式化的文件，不实际格式化
        check: 如果为 True，检查文件是否已格式化，不实际修改文件
    
    Returns:
        退出码（0 表示成功）
    """
    root = get_project_root()
    
    # 检查 .clang-format 文件是否存在
    clang_format_config = root / ".clang-format"
    if not clang_format_config.exists():
        print(f"Error: .clang-format not found at {clang_format_config}", 
              file=sys.stderr)
        return 1
    
    # 默认格式化 autonomy 和 autolink 目录
    if directories is None:
        directories = ["autonomy", "autolink"]
    
    files = list_source_files(root, directories)
    
    if not files:
        print("No C/C++ files found to format.")
        return 0
    
    if dry_run:
        print(f"Found {len(files)} files that would be formatted:")
        for f in files:
            print(f"  {f}")
        return 0
    
    # 实际格式化需要 clang-format
    clang_format = find_clang_format()
    if not clang_format:
        print("Error: clang-format not found in PATH", file=sys.stderr)
        print("Please install clang-format:", file=sys.stderr)
        print("  macOS: brew install clang-format", file=sys.stderr)
        print("  Linux: sudo apt-get install clang-format", file=sys.stderr)
        return 1
    
    print(f"Using clang-format: {clang_format}")
    print(f"Formatting {len(files)} files...")
    
    if check:
        # 检查模式：使用 --dry-run 和 --Werror 来检查格式
        cmd = [clang_format, "--dry-run", "--Werror", "--style=file"] + files
        try:
            subprocess.check_call(cmd, cwd=root)
            print(f"All {len(files)} files are properly formatted.")
            return 0
        except subprocess.CalledProcessError as e:
            print(f"Error: Some files are not properly formatted.", file=sys.stderr)
            return e.returncode
    else:
        # 实际格式化模式
        cmd = [clang_format, "-i", "--style=file"] + files
        try:
            subprocess.check_call(cmd, cwd=root)
            print(f"Successfully formatted {len(files)} files.")
            return 0
        except subprocess.CalledProcessError as e:
            print(f"Error: clang-format failed with exit code {e.returncode}", 
                  file=sys.stderr)
            return e.returncode


def main():
    parser = argparse.ArgumentParser(
        description="Format C/C++ source files using clang-format",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Format all files in autonomy and autolink directories
  python3 scripts/format.py

  # List files that would be formatted (dry run)
  python3 scripts/format.py --dry-run

  # Check if files are properly formatted
  python3 scripts/format.py --check

  # Format only specific directories
  python3 scripts/format.py autonomy
  python3 scripts/format.py autonomy autolink
        """
    )
    
    parser.add_argument(
        "directories",
        nargs="*",
        help="Directories to format (default: autonomy autolink)"
    )
    
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="List files that would be formatted without actually formatting them"
    )
    
    parser.add_argument(
        "--check",
        action="store_true",
        help="Check if files are properly formatted without modifying them"
    )
    
    args = parser.parse_args()
    
    # --dry-run 和 --check 不能同时使用
    if args.dry_run and args.check:
        print("Error: --dry-run and --check cannot be used together", 
              file=sys.stderr)
        return 1
    
    # 如果指定了目录，使用指定的目录；否则使用 None（将使用默认值）
    directories = args.directories if args.directories else None
    
    return format_files(directories=directories, dry_run=args.dry_run, 
                       check=args.check)


if __name__ == "__main__":
    sys.exit(main())
