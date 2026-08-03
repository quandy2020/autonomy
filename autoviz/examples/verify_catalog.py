#!/usr/bin/env python3
"""Verify BICMap example catalog matches publisher, Python script, and gtests."""

from __future__ import annotations

import re
import sys
from pathlib import Path

try:
    import yaml
except ImportError:
    yaml = None  # type: ignore

ROOT = Path(__file__).resolve().parent.parent
CATALOG = Path(__file__).resolve().parent / "catalog.yaml"
PUBLISHER_CPP = ROOT / "tools" / "bicmap_example_publisher.cpp"
PUBLISHER_PY = ROOT / "scripts" / "publish_bicmap_example.py"
GTEST_CPP = ROOT / "tests" / "bicmap_examples_test.cpp"


def load_catalog_names() -> list[str]:
    if yaml is None:
        raise SystemExit("PyYAML required: pip install pyyaml")
    data = yaml.safe_load(CATALOG.read_text(encoding="utf-8"))
    names = [entry["name"] for entry in data["examples"]]
    expected = int(data.get("count", len(names)))
    if len(names) != expected:
        raise AssertionError(f"catalog count mismatch: {len(names)} != {expected}")
    return names


def names_from_cpp_build_examples(path: Path) -> list[str]:
    text = path.read_text(encoding="utf-8")
    block = text.split("BuildExamples()", 1)[1].split("};", 1)[0]
    return re.findall(r'\{"([^"]+)",\s*"/', block)


def names_from_gtest(path: Path) -> list[str]:
    text = path.read_text(encoding="utf-8")
    block = text.split("AllBicMapExamples()", 1)[1].split("};", 1)[0]
    return re.findall(r'\{"/[^"]+",\s*"([^"]+)"', block)


def names_from_python(path: Path) -> list[str]:
    text = path.read_text(encoding="utf-8")
    return re.findall(r'ExampleScene\("([^"]+)"', text)


def main() -> int:
    catalog = load_catalog_names()
    cpp = names_from_cpp_build_examples(PUBLISHER_CPP)
    py = names_from_python(PUBLISHER_PY)
    gtest = names_from_gtest(GTEST_CPP)

    errors: list[str] = []
    for label, names in (
        ("C++ publisher", cpp),
        ("Python publisher", py),
        ("gtest", gtest),
    ):
        if sorted(names) != sorted(catalog):
            missing = sorted(set(catalog) - set(names))
            extra = sorted(set(names) - set(catalog))
            errors.append(
                f"{label}: count={len(names)} catalog={len(catalog)} "
                f"missing={missing} extra={extra}"
            )

    if errors:
        for err in errors:
            print(err, file=sys.stderr)
        return 1

    print(f"OK: {len(catalog)} BICMap examples aligned across catalog, publisher, and tests")
    return 0


if __name__ == "__main__":
    sys.exit(main())
