# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0

"""Parse an ament-style package.xml (format 2 or 3)."""

import argparse
import os
import re
import sys
import xml.etree.ElementTree as ET


class Package:
    def __init__(self, path):
        self.path = path
        self.name = ""
        self.version = ""
        self.description = ""
        self.license = ""
        self.maintainers = []
        self.build_depends = []
        self.buildtool_depends = []
        self.export_depends = []
        self.test_depends = []
        self.version_specs = {}
        self.build_type = ""


def node_text(node):
    if node is None or node.text is None:
        return ""
    return node.text.strip()


def append_unique(items, name):
    name = name.strip()
    if name and name not in items:
        items.append(name)


def tokenize_condition(text):
    tokens = []
    index = 0
    length = len(text)
    while index < length:
        char = text[index]
        if char.isspace():
            index += 1
            continue
        if text.startswith(("==", "!="), index):
            tokens.append(text[index:index + 2])
            index += 2
            continue
        if char in "()":
            tokens.append(char)
            index += 1
            continue
        if char in "'\"":
            quote = char
            index += 1
            start = index
            while index < length and text[index] != quote:
                index += 1
            if index >= length:
                raise ValueError("unterminated string in condition")
            tokens.append(text[start:index])
            index += 1
            continue
        if char == "$":
            index += 1
            start = index
            while index < length and (text[index].isalnum() or text[index] == "_"):
                index += 1
            if start == index:
                raise ValueError("expected a name after $")
            tokens.append(os.environ.get(text[start:index], ""))
            continue
        start = index
        while index < length and (text[index].isalnum() or text[index] == "_"):
            index += 1
        if start == index:
            raise ValueError(f"unexpected {char!r} in condition")
        tokens.append(text[start:index])
    return tokens


def as_bool(value):
    if isinstance(value, bool):
        return value
    raise ValueError("expected a boolean expression")


def parse_primary(tokens, index):
    if index >= len(tokens):
        raise ValueError("unexpected end of condition")
    token = tokens[index]
    if token == "(":
        value, index = parse_or(tokens, index + 1)
        if index >= len(tokens) or tokens[index] != ")":
            raise ValueError("missing closing parenthesis")
        return value, index + 1
    if token == "true":
        return True, index + 1
    if token == "false":
        return False, index + 1
    if token in ("and", "or", "not", "==", "!=", ")"):
        raise ValueError(f"unexpected {token}")
    return token, index + 1


def parse_compare(tokens, index):
    value, index = parse_primary(tokens, index)
    if index < len(tokens) and tokens[index] in ("==", "!="):
        operator = tokens[index]
        right, index = parse_primary(tokens, index + 1)
        same = str(value) == str(right)
        return (same if operator == "==" else not same), index
    return value, index


def parse_not(tokens, index):
    if index < len(tokens) and tokens[index] == "not":
        value, index = parse_not(tokens, index + 1)
        return (not as_bool(value)), index
    return parse_compare(tokens, index)


def parse_and(tokens, index):
    value, index = parse_not(tokens, index)
    while index < len(tokens) and tokens[index] == "and":
        right, index = parse_not(tokens, index + 1)
        value = as_bool(value) and as_bool(right)
    return value, index


def parse_or(tokens, index):
    value, index = parse_and(tokens, index)
    while index < len(tokens) and tokens[index] == "or":
        right, index = parse_and(tokens, index + 1)
        value = as_bool(value) or as_bool(right)
    return value, index


def evaluate_condition(expression):
    tokens = tokenize_condition(expression)
    value, index = parse_or(tokens, 0)
    if index != len(tokens):
        raise ValueError("trailing tokens in condition")
    return as_bool(value)


def dependency_selected(node, path):
    condition = node.attrib.get("condition", "").strip()
    if not condition:
        return True
    try:
        return evaluate_condition(condition)
    except ValueError as exc:
        raise ValueError(f"{path}: <{node.tag}> condition: {exc}") from exc


def version_constraint(node, path):
    found = [key for key in ("version_eq", "version_gte", "version_gt", "version_lte", "version_lt") if key in node.attrib]
    if not found:
        return "", ""
    if len(found) > 1:
        raise ValueError(f"{path}: <{node.tag}> '{node_text(node)}' has more than one version attribute")
    version = node.attrib[found[0]].strip()
    if not re.fullmatch(r"\d+\.\d+(\.\d+)?", version):
        raise ValueError(
            f"{path}: <{node.tag}> '{node_text(node)}' version '{version}' is not MAJOR.MINOR or MAJOR.MINOR.PATCH")
    return found[0][len("version_"):], version


def add_dependency(package, items, node, path):
    if not dependency_selected(node, path):
        return
    name = node_text(node)
    if not name:
        return
    kind, version = version_constraint(node, path)
    current = package.version_specs.get(name)
    if current and (kind, version) not in (current, ("", "")):
        raise ValueError(f"{path}: '{name}' has conflicting version constraints")
    if kind and current != (kind, version):
        package.version_specs[name] = (kind, version)
    append_unique(items, name)


def parse_package_xml(path):
    try:
        root = ET.parse(path).getroot()
    except ET.ParseError as exc:
        raise ValueError(f"{path}: {exc}") from exc
    if root.tag != "package":
        raise ValueError(f"{path}: root element is '{root.tag}', expected 'package'")

    package = Package(path)
    package.name = node_text(root.find("name"))
    package.version = node_text(root.find("version"))
    package.description = " ".join(node_text(root.find("description")).split())
    package.license = node_text(root.find("license"))
    for node in root.findall("maintainer"):
        email = node.attrib.get("email", "")
        label = node_text(node)
        package.maintainers.append(f"{label} <{email}>" if email else label)

    if not package.name:
        raise ValueError(f"{path}: <name> is required")
    if not package.version:
        raise ValueError(f"{path}: <version> is required")
    if not re.fullmatch(r"\d+\.\d+\.\d+", package.version):
        raise ValueError(f"{path}: <version> '{package.version}' is not MAJOR.MINOR.PATCH")
    if root.findall("group_depend"):
        raise ValueError(f"{path}: <group_depend> is not supported")

    for node in root.findall("buildtool_depend"):
        add_dependency(package, package.buildtool_depends, node, path)
    for node in root.findall("build_depend"):
        add_dependency(package, package.build_depends, node, path)
    for node in root.findall("test_depend"):
        add_dependency(package, package.test_depends, node, path)
    for node in root.findall("exec_depend"):
        add_dependency(package, package.export_depends, node, path)
    for node in root.findall("build_export_depend"):
        add_dependency(package, package.export_depends, node, path)
    for node in root.findall("buildtool_export_depend"):
        add_dependency(package, package.export_depends, node, path)
    for node in root.findall("depend"):
        if dependency_selected(node, path):
            add_dependency(package, package.build_depends, node, path)
            add_dependency(package, package.export_depends, node, path)

    export = root.find("export")
    if export is not None:
        package.build_type = node_text(export.find("build_type"))
    return package


def cmake_string(value):
    return (
        value.replace("\\", "\\\\")
        .replace('"', '\\"')
        .replace("\n", "\\n")
        .replace("${", "\\${")
    )


def cmake_list(items):
    return ";".join(items)


def write_cmake(package, destination):
    name = package.name
    specs = []
    for dep, (kind, version) in package.version_specs.items():
        specs.extend((dep, kind, version))
    lines = [
        f'set(_AUTOCMAKE_PACKAGE_NAME "{cmake_string(name)}")',
        f'set({name}_VERSION "{cmake_string(package.version)}")',
        f'set({name}_DESCRIPTION "{cmake_string(package.description)}")',
        f'set({name}_LICENSE "{cmake_string(package.license)}")',
        f'set({name}_MAINTAINER "{cmake_string(", ".join(package.maintainers))}")',
        f'set({name}_BUILD_DEPENDS "{cmake_list(package.build_depends)}")',
        f'set({name}_BUILDTOOL_DEPENDS "{cmake_list(package.buildtool_depends)}")',
        f'set({name}_EXPORT_DEPENDS "{cmake_list(package.export_depends)}")',
        f'set({name}_TEST_DEPENDS "{cmake_list(package.test_depends)}")',
        f'set({name}_VERSION_SPECS "{cmake_list(specs)}")',
        f'set({name}_BUILD_TYPE "{cmake_string(package.build_type)}")',
        "",
    ]
    with open(destination, "w", encoding="utf-8") as handle:
        handle.write("\n".join(lines))


def main(argv):
    parser = argparse.ArgumentParser(description="Parse package.xml into a CMake snippet")
    parser.add_argument("--cmake", nargs=2, metavar=("PACKAGE_XML", "OUTPUT"))
    args = parser.parse_args(argv)
    if not args.cmake:
        parser.error("pass --cmake PACKAGE_XML OUTPUT")
    try:
        package = parse_package_xml(args.cmake[0])
    except ValueError as exc:
        print(exc, file=sys.stderr)
        return 1
    write_cmake(package, args.cmake[1])
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
