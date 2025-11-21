#!/usr/bin/env python3
"""
Autolink 构建脚本
支持 build, clean, test, install, format 等操作
"""

import argparse
import subprocess
import sys
import os
import platform
import shutil
from pathlib import Path


def get_project_root():
    """获取项目根目录"""
    script_dir = Path(__file__).parent.absolute()
    return script_dir.parent


def detect_platform():
    """检测当前平台"""
    system = platform.system()
    if system == "Darwin":
        return "macos"
    elif system == "Linux":
        return "linux"
    else:
        return None


def get_platform_config():
    """获取平台特定的 Bazel 配置"""
    platform_name = detect_platform()
    if platform_name:
        return f"--config={platform_name}"
    return None


def run_command(cmd, cwd=None, check=True):
    """运行命令并返回结果"""
    if cwd is None:
        cwd = get_project_root()
    
    print(f"Running: {' '.join(cmd)}")
    result = subprocess.run(cmd, cwd=cwd, check=check)
    return result


def build(targets=None, verbose=False, debug=False):
    """构建目标"""
    cmd = ["bazel", "build"]
    
    # 自动添加平台特定配置
    platform_config = get_platform_config()
    if platform_config:
        cmd.append(platform_config)
        print(f"Using platform config: {platform_config}")
    
    # 添加调试配置
    if debug:
        cmd.append("--config=dbg")
        print("Using debug config: --config=dbg")
    
    if verbose:
        cmd.append("--verbose_failures")
    
    if targets:
        cmd.extend(targets)
    else:
        cmd.append("//...")
    
    run_command(cmd)


def clean(expunge=False):
    """清理构建产物"""
    cmd = ["bazel", "clean"]
    
    if expunge:
        cmd.append("--expunge")
        print("Warning: This will remove all build artifacts and caches!")
    
    run_command(cmd)


def test(targets=None, verbose=False, debug=False):
    """运行测试"""
    cmd = ["bazel", "test"]
    
    # 自动添加平台特定配置
    platform_config = get_platform_config()
    if platform_config:
        cmd.append(platform_config)
    
    # 添加调试配置
    if debug:
        cmd.append("--config=dbg")
        print("Using debug config: --config=dbg")
    
    if verbose:
        cmd.append("--test_output=all")
    else:
        cmd.append("--test_output=errors")
    
    if targets:
        cmd.extend(targets)
    else:
        cmd.append("//...")
    
    run_command(cmd)


def install(target=None, install_path=None, libdir=None, includedir=None, docdir=None, cmakedir=None, install_libs=False, install_headers=False, install_docs=False, install_cmake=False):
    """安装构建产物（可执行文件、库文件、头文件、文档、CMake配置文件）"""
    # 如果没有指定 target，默认使用 autolink 库
    if not target:
        target = "//autolink:autolink"
    
    project_root = get_project_root()
    
    # 构建目标
    print(f"Building target: {target}")
    build([target])
    
    # 获取 bazel-bin 路径
    bazel_bin_result = subprocess.run(
        ["bazel", "info", "bazel-bin"],
        capture_output=True,
        text=True,
        check=True,
        cwd=project_root
    )
    bazel_bin = Path(bazel_bin_result.stdout.strip())
    
    # 确定安装路径
    if not install_path:
        install_path = os.path.expanduser("~/bin")
    install_path = Path(install_path)
    
    # 确定库目录和头文件目录
    if not libdir:
        libdir = install_path.parent / "lib"
    else:
        libdir = Path(libdir)
    
    if not includedir:
        includedir = install_path.parent / "include"
    else:
        includedir = Path(includedir)
    
    # 获取构建产物路径
    result = subprocess.run(
        ["bazel", "cquery", target, "--output=files"],
        capture_output=True,
        text=True,
        check=True,
        cwd=project_root
    )
    
    output_files = [f.strip() for f in result.stdout.strip().split('\n') if f.strip()]
    
    installed_files = []
    
    # 处理可执行文件和库文件
    for output_file in output_files:
        file_path = Path(output_file)
        
        # 检查是否是库文件（.so, .dylib, .a）
        if file_path.suffix in ['.so', '.dylib', '.a'] or 'lib' in file_path.name.lower():
            if install_libs:
                libdir.mkdir(parents=True, exist_ok=True)
                dest_path = libdir / file_path.name
                shutil.copy2(file_path, dest_path)
                installed_files.append(("library", dest_path))
                print(f"Installed library: {dest_path}")
        # 检查是否是可执行文件
        elif os.access(file_path, os.X_OK) or file_path.suffix == '':
            install_path.mkdir(parents=True, exist_ok=True)
            dest_path = install_path / file_path.name
            shutil.copy2(file_path, dest_path)
            os.chmod(dest_path, 0o755)
            installed_files.append(("binary", dest_path))
            print(f"Installed binary: {dest_path}")
    
    # 安装头文件
    if install_headers:
        # 查找所有头文件
        target_parts = target.replace("//", "").split(":")
        package_path = target_parts[0] if target_parts else ""
        
        # 获取目标的头文件
        # 使用 bazel query 查找目标的 hdrs
        query_result = subprocess.run(
            ["bazel", "query", f"labels(hdrs, {target})"],
            capture_output=True,
            text=True,
            check=False,
            cwd=project_root
        )
        
        if query_result.returncode == 0:
            header_targets = [t.strip() for t in query_result.stdout.strip().split('\n') if t.strip()]
            
            # 查找源目录中的头文件
            source_dirs = []
            if package_path:
                source_dir = project_root / package_path
                if source_dir.exists():
                    source_dirs.append(source_dir)
            
            # 也检查 autolink 目录下的所有头文件
            autolink_dir = project_root / "autolink"
            if autolink_dir.exists():
                source_dirs.append(autolink_dir)
            
            includedir.mkdir(parents=True, exist_ok=True)
            
            # 复制头文件，保持目录结构
            for source_dir in source_dirs:
                for header_file in source_dir.rglob("*.hpp"):
                    rel_path = header_file.relative_to(project_root)
                    dest_path = includedir / rel_path
                    dest_path.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(header_file, dest_path)
                    installed_files.append(("header", dest_path))
                    print(f"Installed header: {dest_path}")
                
                for header_file in source_dir.rglob("*.h"):
                    rel_path = header_file.relative_to(project_root)
                    dest_path = includedir / rel_path
                    dest_path.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(header_file, dest_path)
                    installed_files.append(("header", dest_path))
                    print(f"Installed header: {dest_path}")
    
    # 安装文档和 CMake 配置文件（在摘要之前安装，以便包含在摘要中）
    if install_docs:
        docs_source = project_root / "docs"
        if docs_source.exists():
            docdir.mkdir(parents=True, exist_ok=True)
            
            # 复制所有文档文件
            doc_files = []
            for doc_file in docs_source.rglob("*"):
                if doc_file.is_file() and not doc_file.name.startswith('.'):
                    rel_path = doc_file.relative_to(docs_source)
                    dest_path = docdir / rel_path
                    dest_path.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(doc_file, dest_path)
                    doc_files.append(dest_path)
                    print(f"Installed doc: {dest_path}")
            
            if doc_files:
                installed_files.append(("doc", docdir))
        else:
            print("Warning: docs directory not found, skipping documentation installation")
    
    # 安装 CMake 配置文件
    if install_cmake:
        cmake_source = project_root / "cmake"
        if cmake_source.exists():
            cmakedir.mkdir(parents=True, exist_ok=True)
            
            # 复制所有 CMake 配置文件
            cmake_files = []
            for cmake_file in cmake_source.rglob("*"):
                if cmake_file.is_file() and not cmake_file.name.startswith('.'):
                    rel_path = cmake_file.relative_to(cmake_source)
                    dest_path = cmakedir / rel_path
                    dest_path.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(cmake_file, dest_path)
                    cmake_files.append(dest_path)
                    print(f"Installed cmake config: {dest_path}")
            
            if cmake_files:
                installed_files.append(("cmake", cmakedir))
        else:
            print("Warning: cmake directory not found, skipping CMake config installation")
    
    # 输出安装摘要
    if installed_files:
        print("\n" + "="*60)
        print("Installation Summary:")
        print("="*60)
        binaries = [f for t, f in installed_files if t == "binary"]
        libraries = [f for t, f in installed_files if t == "library"]
        headers = [f for t, f in installed_files if t == "header"]
        docs = [f for t, f in installed_files if t == "doc"]
        cmake_configs = [f for t, f in installed_files if t == "cmake"]
        
        if binaries:
            print(f"\nBinaries installed to: {install_path}")
            for f in binaries:
                print(f"  - {f.name}")
            print(f"\nAdd {install_path} to your PATH to use binaries")
        
        if libraries:
            print(f"\nLibraries installed to: {libdir}")
            for f in libraries:
                print(f"  - {f.name}")
            print(f"\nAdd {libdir} to your LD_LIBRARY_PATH (Linux) or DYLD_LIBRARY_PATH (macOS)")
        
        if headers:
            print(f"\nHeaders installed to: {includedir}")
            print(f"  Total: {len(headers)} header files")
            print(f"\nUse -I{includedir} when compiling")
        
        if docs:
            print(f"\nDocumentation installed to: {docdir}")
            doc_count = len([f for f in docdir.rglob("*") if f.is_file()])
            print(f"  Total: {doc_count} documentation files")
        
        if cmake_configs:
            print(f"\nCMake config files installed to: {cmakedir}")
            cmake_count = len([f for f in cmakedir.rglob("*") if f.is_file()])
            print(f"  Total: {cmake_count} CMake config files")
            print(f"\nAdd {cmakedir} to CMAKE_PREFIX_PATH or CMAKE_MODULE_PATH to use")
        
        print("="*60)
    
    if not installed_files:
        print("Warning: No files were installed")


def run(target, args=None, debug=False):
    """运行可执行目标"""
    cmd = ["bazel", "run"]
    
    # 自动添加平台特定配置
    platform_config = get_platform_config()
    if platform_config:
        cmd.append(platform_config)
    
    # 添加调试配置
    if debug:
        cmd.append("--config=dbg")
        print("Using debug config: --config=dbg")
    
    cmd.append(target)
    
    if args:
        cmd.append("--")
        cmd.extend(args)
    
    run_command(cmd)


def query(query_expr):
    """查询 Bazel 目标"""
    cmd = ["bazel", "query", query_expr]
    result = subprocess.run(
        cmd,
        cwd=get_project_root(),
        check=True
    )
    return result


def find_clang_format():
    """查找 clang-format 工具"""
    # 首先尝试标准 PATH 中的 clang-format（包括版本化名称）
    for name in ("clang-format", "clang-format-17", "clang-format-16", "clang-format-15", "clang-format-14"):
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


def list_source_files(root_dir=None):
    """列出项目中的 C/C++ 源文件"""
    if root_dir is None:
        root_dir = get_project_root()
    
    exts = {".c", ".cc", ".cpp", ".h", ".hpp"}
    files = []
    
    # 只查找 autolink 目录
    autolink_dir = Path(root_dir) / "autolink"
    if autolink_dir.exists() and autolink_dir.is_dir():
        for source_file in autolink_dir.rglob("*"):
            if source_file.is_file() and source_file.suffix in exts:
                files.append(str(source_file))
    
    return files


def format_files(dry_run=False):
    """格式化项目中的 C/C++ 源文件"""
    root = get_project_root()
    files = list_source_files(root)
    
    if not files:
        print("No C/C++ files found to format.")
        return 0
    
    # dry-run 模式下不需要 clang-format
    if dry_run:
        print(f"Found {len(files)} files that would be formatted:")
        for f in files:
            print(f"  {f}")
        return 0
    
    # 实际格式化需要 clang-format
    clang_format = find_clang_format()
    if not clang_format:
        print("Error: clang-format not found in PATH", file=sys.stderr)
        print("Please install clang-format (e.g., brew install clang-format)", file=sys.stderr)
        return 1
    
    print(f"Formatting {len(files)} files with {clang_format}...")
    cmd = [clang_format, "-i"] + files
    try:
        subprocess.check_call(cmd, cwd=root)
        print(f"Successfully formatted {len(files)} files.")
        return 0
    except subprocess.CalledProcessError as e:
        print(f"Error: clang-format failed with exit code {e.returncode}", file=sys.stderr)
        return e.returncode


def main():
    parser = argparse.ArgumentParser(
        description="Autolink 构建脚本",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  %(prog)s build                             # 构建所有目标
  %(prog)s build //autolink/examples:talker  # 构建特定目标
  %(prog)s clean                             # 清理构建产物
  %(prog)s clean --expunge                   # 完全清理（包括缓存）
  %(prog)s test                              # 运行所有测试
  %(prog)s test //autolink/common:log_test   # 运行特定测试
  %(prog)s install --prefix /usr/local       # 安装 autolink 库、头文件和可执行文件（默认）
  %(prog)s install --prefix /usr/local --install-docs --install-cmake  # 同时安装文档和 CMake 配置
  %(prog)s install //autolink/examples:talker  # 安装特定可执行文件
  %(prog)s install --prefix /usr/local --no-headers  # 只安装库文件，不安装头文件
  %(prog)s run //autolink/examples:talker            # 运行可执行文件
  %(prog)s query 'kind(cc_binary, //...)'            # 查询所有二进制目标
  %(prog)s format                                    # 格式化所有 C/C++ 源文件
  %(prog)s format --dry-run                          # 查看将被格式化的文件列表
        """
    )
    
    subparsers = parser.add_subparsers(dest="command", help="可用命令")
    
    # build 命令
    build_parser = subparsers.add_parser("build", help="构建目标")
    build_parser.add_argument(
        "targets",
        nargs="*",
        help="要构建的目标（默认为 //...）"
    )
    build_parser.add_argument(
        "-v", "--verbose",
        action="store_true",
        help="显示详细输出"
    )
    build_parser.add_argument(
        "-d", "--debug",
        action="store_true",
        help="使用调试模式构建（包含调试符号，无优化）"
    )
    
    # clean 命令
    clean_parser = subparsers.add_parser("clean", help="清理构建产物")
    clean_parser.add_argument(
        "--expunge",
        action="store_true",
        help="完全清理（包括所有缓存）"
    )
    
    # test 命令
    test_parser = subparsers.add_parser("test", help="运行测试")
    test_parser.add_argument(
        "targets",
        nargs="*",
        help="要测试的目标（默认为 //...）"
    )
    test_parser.add_argument(
        "-v", "--verbose",
        action="store_true",
        help="显示所有测试输出"
    )
    test_parser.add_argument(
        "-d", "--debug",
        action="store_true",
        help="使用调试模式运行测试（包含调试符号，无优化）"
    )
    
    # install 命令
    install_parser = subparsers.add_parser("install", help="安装构建产物（可执行文件、库文件、头文件）")
    install_parser.add_argument(
        "target",
        nargs="?",
        default=None,
        help="要安装的目标（默认：//autolink:autolink，例如：//autolink/examples:talker）"
    )
    install_parser.add_argument(
        "--prefix",
        default=None,
        help="安装前缀路径（默认为 ~，会安装到 ~/bin, ~/lib, ~/include）"
    )
    install_parser.add_argument(
        "--bindir",
        default=None,
        help="可执行文件安装目录（默认：<prefix>/bin）"
    )
    install_parser.add_argument(
        "--libdir",
        default=None,
        help="库文件安装目录（默认：<prefix>/lib）"
    )
    install_parser.add_argument(
        "--includedir",
        default=None,
        help="头文件安装目录（默认：<prefix>/include）"
    )
    install_parser.add_argument(
        "--install-libs",
        action="store_true",
        help="安装库文件（使用 --prefix 时默认启用）"
    )
    install_parser.add_argument(
        "--install-headers",
        action="store_true",
        help="安装头文件（使用 --prefix 时默认启用）"
    )
    install_parser.add_argument(
        "--no-libs",
        action="store_true",
        dest="no_libs",
        help="不安装库文件（与 --prefix 一起使用时禁用库文件安装）"
    )
    install_parser.add_argument(
        "--no-headers",
        action="store_true",
        dest="no_headers",
        help="不安装头文件（与 --prefix 一起使用时禁用头文件安装）"
    )
    install_parser.add_argument(
        "--docdir",
        default=None,
        help="文档安装目录（默认：<prefix>/share/doc/autolink）"
    )
    install_parser.add_argument(
        "--install-docs",
        action="store_true",
        dest="install_docs",
        help="同时安装文档（使用 --prefix 时默认不安装）"
    )
    install_parser.add_argument(
        "--cmakedir",
        default=None,
        help="CMake 配置文件安装目录（默认：<prefix>/lib/cmake/autolink）"
    )
    install_parser.add_argument(
        "--install-cmake",
        action="store_true",
        dest="install_cmake",
        help="同时安装 CMake 配置文件（使用 --prefix 时默认不安装）"
    )
    
    # run 命令
    run_parser = subparsers.add_parser("run", help="运行可执行目标")
    run_parser.add_argument(
        "target",
        help="要运行的目标（例如：//autolink/examples:talker）"
    )
    run_parser.add_argument(
        "args",
        nargs=argparse.REMAINDER,
        help="传递给可执行文件的参数"
    )
    run_parser.add_argument(
        "-d", "--debug",
        action="store_true",
        help="使用调试模式运行（包含调试符号，无优化）"
    )
    
    # query 命令
    query_parser = subparsers.add_parser("query", help="查询 Bazel 目标")
    query_parser.add_argument(
        "query_expr",
        help="查询表达式（例如：'kind(cc_binary, //...)'）"
    )
    
    # format 命令
    format_parser = subparsers.add_parser("format", help="格式化 C/C++ 源代码")
    format_parser.add_argument(
        "--dry-run",
        action="store_true",
        help="只显示将被格式化的文件，不实际格式化"
    )
    
    args = parser.parse_args()
    
    if not args.command:
        parser.print_help()
        sys.exit(1)
    
    try:
        if args.command == "build":
            build(args.targets if args.targets else None, args.verbose, getattr(args, 'debug', False))
        elif args.command == "clean":
            clean(args.expunge)
        elif args.command == "test":
            test(args.targets if args.targets else None, args.verbose, getattr(args, 'debug', False))
        elif args.command == "install":
            # 处理安装路径
            if args.prefix:
                prefix = Path(args.prefix)
                bindir = args.bindir if args.bindir else prefix / "bin"
                libdir = args.libdir if args.libdir else prefix / "lib"
                includedir = args.includedir if args.includedir else prefix / "include"
                docdir = args.docdir if args.docdir else prefix / "share" / "doc" / "autolink"
                cmakedir = args.cmakedir if args.cmakedir else prefix / "lib" / "cmake" / "autolink"
                # 当指定 --prefix 时，默认安装库和头文件
                # 用户可以通过 --no-libs 或 --no-headers 禁用
                install_libs = not getattr(args, 'no_libs', False)
                install_headers = not getattr(args, 'no_headers', False)
                # 文档和 CMake 配置需要明确指定才安装
                install_docs = getattr(args, 'install_docs', False)
                install_cmake = getattr(args, 'install_cmake', False)
            else:
                bindir = args.bindir if args.bindir else None
                libdir = args.libdir if args.libdir else None
                includedir = args.includedir if args.includedir else None
                docdir = args.docdir if args.docdir else None
                cmakedir = args.cmakedir if args.cmakedir else None
                # 未指定 --prefix 时，只有明确指定才安装
                install_libs = args.install_libs
                install_headers = args.install_headers
                install_docs = getattr(args, 'install_docs', False)
                install_cmake = getattr(args, 'install_cmake', False)
            
            install(
                args.target,
                install_path=bindir,
                libdir=libdir,
                includedir=includedir,
                docdir=docdir,
                cmakedir=cmakedir,
                install_libs=install_libs,
                install_headers=install_headers,
                install_docs=install_docs,
                install_cmake=install_cmake
            )
        elif args.command == "run":
            run(args.target, args.args, getattr(args, 'debug', False))
        elif args.command == "query":
            query(args.query_expr)
        elif args.command == "format":
            sys.exit(format_files(getattr(args, 'dry_run', False)))
    except subprocess.CalledProcessError as e:
        sys.exit(e.returncode)
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        sys.exit(130)


if __name__ == "__main__":
    main()

