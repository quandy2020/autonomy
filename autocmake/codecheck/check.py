#!/usr/bin/env python3
# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0

"""Run package linters and the codecheck target.

lint runs one tool named by package.xml test_depend.
codecheck runs cppcheck (with the rule files in this directory) and cpplint.
"""

import argparse
import os
import shutil
import subprocess
import sys

SKIP_DIRECTORIES = {"build", "install", "log", "Testing", "CMakeFiles", ".git"}
CPP_EXTENSIONS = {".c", ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp", ".hxx"}
PYTHON_EXTENSIONS = {".py"}

LINT_TOOLS = {
    "clang-format": "clang-format",
    "cppcheck": "cppcheck",
    "cpplint": "cpplint",
    "flake8": "flake8",
    "pycodestyle": "pycodestyle",
    "pyflakes": "pyflakes",
    "pep257": "pep257",
    "mypy": "mypy",
    "xmllint": "xmllint",
    "cmakelint": "cmakelint",
    "uncrustify": "uncrustify",
    "clang-tidy": "clang-tidy",
    "copyright": "copyright",
}


def collect_files(root, predicate):
    found = []
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = [
            name for name in dirnames
            if name not in SKIP_DIRECTORIES and not name.startswith(".")
        ]
        for name in filenames:
            path = os.path.join(dirpath, name)
            if predicate(name, path):
                found.append(path)
    return sorted(found)


def cpp_sources(root):
    return collect_files(
        root, lambda name, path: os.path.splitext(name)[1] in CPP_EXTENSIONS)


def python_sources(root):
    return collect_files(
        root, lambda name, path: os.path.splitext(name)[1] in PYTHON_EXTENSIONS)


def xml_sources(root):
    return collect_files(root, lambda name, path: name.endswith(".xml"))


def cmake_sources(root):
    return collect_files(
        root,
        lambda name, path: name == "CMakeLists.txt" or name.endswith(".cmake"))


def require_tool(names):
    if isinstance(names, str):
        names = [names]
    for name in names:
        path = shutil.which(name)
        if path:
            return path
    print("not installed: " + ", ".join(names), file=sys.stderr)
    return None


def run_command(command):
    result = subprocess.call(command)
    return result


def run_in_batches(command, files, batch_size=40):
    if not files:
        return 0
    status = 0
    for start in range(0, len(files), batch_size):
        code = run_command(command + files[start:start + batch_size])
        if code != 0:
            status = code
    return status


def rule_files():
    directory = os.path.dirname(os.path.abspath(__file__))
    return sorted(
        os.path.join(directory, name)
        for name in os.listdir(directory)
        if name.endswith(".rule")
    )


def cppcheck_supports_rules(tool):
    result = subprocess.run(
        [tool, "--help"], capture_output=True, text=True, check=False)
    text = result.stdout + result.stderr
    return "--rule-file" in text


def suppressions_list(package):
    for candidate in (
        os.path.join(package, "cppcheck.suppress"),
        os.path.join(os.getcwd(), "cppcheck.suppress"),
    ):
        if os.path.isfile(candidate):
            return candidate
    return None


def cppcheck_command(tool, package, includes, style):
    command = [
        tool, "-q", "--inline-suppr", "-j", "4",
        "--language=c++", "--std=c++17", "--force", "--error-exitcode=1",
    ]
    if style:
        command.append("--enable=style,performance,portability,information")
        if cppcheck_supports_rules(tool):
            for rule in rule_files():
                command.append("--rule-file=" + rule)
    else:
        command.append("--enable=missingInclude")
    suppressions = suppressions_list(package)
    if suppressions:
        command.append("--suppressions-list=" + suppressions)
    for include in includes:
        command.extend(["-I", include])
    return command


def run_cppcheck(package, includes, both_passes):
    tool = require_tool("cppcheck")
    if not tool:
        return 1
    files = cpp_sources(package)
    if not files:
        return 0
    status = run_in_batches(cppcheck_command(tool, package, includes, True), files)
    if both_passes:
        missing = run_in_batches(
            cppcheck_command(tool, package, includes, False), files)
        if missing != 0:
            status = missing
    return status


def run_cpplint(package, optional):
    tool = require_tool("cpplint")
    if not tool:
        if optional:
            print("cpplint was not found; skipping", file=sys.stderr)
            return 0
        return 1
    files = cpp_sources(package)
    if not files:
        return 0
    return run_in_batches(
        [tool, "--extensions=c,cc,cpp,cxx,h,hh,hpp,hxx", "--quiet"], files)


def run_clang_format(package):
    tool = require_tool("clang-format")
    if not tool:
        return 1
    files = cpp_sources(package)
    return run_in_batches([tool, "--dry-run", "--Werror"], files)


def run_python_tool(package, names, extra):
    tool = require_tool(names)
    if not tool:
        return 1
    files = python_sources(package)
    if not files:
        return 0
    return run_in_batches([tool, *extra], files)


def run_xmllint(package):
    tool = require_tool("xmllint")
    if not tool:
        return 1
    files = xml_sources(package)
    if not files:
        return 0
    return run_in_batches([tool, "--noout"], files)


def run_cmakelint(package):
    tool = require_tool("cmakelint")
    if not tool:
        return 1
    files = cmake_sources(package)
    if not files:
        return 0
    return run_in_batches([tool], files)


def run_uncrustify(package):
    tool = require_tool("uncrustify")
    if not tool:
        return 1
    config = None
    for name in ("uncrustify.cfg", ".uncrustify.cfg"):
        candidate = os.path.join(package, name)
        if os.path.isfile(candidate):
            config = candidate
            break
    if not config:
        print("uncrustify.cfg was not found in the package", file=sys.stderr)
        return 1
    files = cpp_sources(package)
    if not files:
        return 0
    return run_in_batches([tool, "-c", config, "--check"], files)


def run_clang_tidy(package):
    tool = require_tool("clang-tidy")
    if not tool:
        return 1
    database = None
    for candidate in (
        os.path.join(os.getcwd(), "compile_commands.json"),
        os.path.join(package, "compile_commands.json"),
    ):
        if os.path.isfile(candidate):
            database = os.path.dirname(candidate)
            break
    if not database:
        print("compile_commands.json was not found", file=sys.stderr)
        return 1
    files = [
        path for path in cpp_sources(package)
        if os.path.splitext(path)[1] in {".c", ".cc", ".cpp", ".cxx"}
    ]
    if not files:
        return 0
    return run_in_batches([tool, "-p", database], files)


def run_copyright(package):
    files = cpp_sources(package) + python_sources(package) + cmake_sources(package)
    missing = []
    for path in files:
        with open(path, encoding="utf-8", errors="replace") as handle:
            head = "".join(handle.readline() for _ in range(40))
        if "Copyright" not in head:
            missing.append(path)
    if missing:
        print("missing Copyright header:", file=sys.stderr)
        for path in missing:
            print(path, file=sys.stderr)
        return 1
    return 0


def run_lint(tool, package, includes):
    if tool == "clang-format":
        return run_clang_format(package)
    if tool == "cppcheck":
        return run_cppcheck(package, includes, False)
    if tool == "cpplint":
        return run_cpplint(package, False)
    if tool == "flake8":
        return run_python_tool(package, "flake8", [])
    if tool == "pycodestyle":
        return run_python_tool(package, "pycodestyle", [])
    if tool == "pyflakes":
        return run_python_tool(package, "pyflakes", [])
    if tool == "pep257":
        return run_python_tool(package, ["pydocstyle", "pep257"], [])
    if tool == "mypy":
        return run_python_tool(package, "mypy", [])
    if tool == "xmllint":
        return run_xmllint(package)
    if tool == "cmakelint":
        return run_cmakelint(package)
    if tool == "uncrustify":
        return run_uncrustify(package)
    if tool == "clang-tidy":
        return run_clang_tidy(package)
    if tool == "copyright":
        return run_copyright(package)
    print("unknown lint tool: " + tool, file=sys.stderr)
    return 2


def run_codecheck(package, includes):
    status = run_cppcheck(package, includes, True)
    cpplint_status = run_cpplint(package, True)
    if cpplint_status != 0:
        status = cpplint_status
    return status


def main(argv):
    parser = argparse.ArgumentParser(description="autocmake lint and codecheck")
    sub = parser.add_subparsers(dest="command", required=True)

    lint = sub.add_parser("lint")
    lint.add_argument("tool", choices=sorted(LINT_TOOLS))
    lint.add_argument("package")
    lint.add_argument("--include", action="append", default=[])

    codecheck = sub.add_parser("codecheck")
    codecheck.add_argument("package")
    codecheck.add_argument("--include", action="append", default=[])

    args = parser.parse_args(argv)
    package = os.path.abspath(args.package)
    if args.command == "lint":
        return run_lint(args.tool, package, args.include)
    return run_codecheck(package, args.include)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
