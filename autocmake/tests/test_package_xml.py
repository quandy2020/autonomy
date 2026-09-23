# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Checks for package.xml parsing, conditions, and CMake escaping."""

import importlib.util
import os
import sys
import tempfile
import unittest
from pathlib import Path


def load_module(path):
    spec = importlib.util.spec_from_file_location("package_xml", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


XML = load_module(sys.argv[1])


def write_package(directory, body):
    path = Path(directory) / "package.xml"
    path.write_text(body, encoding="utf-8")
    return path


class PackageXmlTest(unittest.TestCase):
    def test_depend_roles_and_cmake_escape(self):
        with tempfile.TemporaryDirectory() as directory:
            path = write_package(directory, """<?xml version="1.0"?>
<package format="3">
  <name>sample</name>
  <version>1.2.3</version>
  <description>keep ${HOME} and "quotes"</description>
  <maintainer email="dev@example.com">autocmake</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>autocmake</buildtool_depend>
  <buildtool_export_depend>cmake</buildtool_export_depend>
  <build_depend>private_lib</build_depend>
  <depend>public_lib</depend>
  <exec_depend>runtime_lib</exec_depend>
  <test_depend>gtest</test_depend>
  <export><build_type>autocmake</build_type></export>
</package>
""")
            package = XML.parse_package_xml(path)
            destination = Path(directory) / "package.cmake"
            XML.write_cmake(package, destination)
            text = destination.read_text(encoding="utf-8")
        self.assertEqual(package.build_depends, ["private_lib", "public_lib"])
        self.assertEqual(package.export_depends, ["runtime_lib", "cmake", "public_lib"])
        self.assertEqual(package.test_depends, ["gtest"])
        self.assertIn(r"keep \${HOME}", text)
        self.assertIn(r"\"quotes\"", text)
        self.assertIn('set(sample_VERSION_SPECS "")', text)

    def test_version_constraints(self):
        with tempfile.TemporaryDirectory() as directory:
            path = write_package(directory, """<?xml version="1.0"?>
<package format="3">
  <name>sample</name>
  <version>1.2.3</version>
  <description>versions</description>
  <maintainer email="dev@example.com">autocmake</maintainer>
  <license>Apache-2.0</license>
  <depend version_gte="1.2.0">public_lib</depend>
  <exec_depend version_eq="2.0">runtime_lib</exec_depend>
  <build_depend version_lt="4.0.0">private_lib</build_depend>
</package>
""")
            package = XML.parse_package_xml(path)
            destination = Path(directory) / "package.cmake"
            XML.write_cmake(package, destination)
            text = destination.read_text(encoding="utf-8")
            conflict = write_package(directory, """<?xml version="1.0"?>
<package format="3">
  <name>sample</name>
  <version>1.2.3</version>
  <description>versions</description>
  <maintainer email="dev@example.com">autocmake</maintainer>
  <license>Apache-2.0</license>
  <build_depend version_gte="1.0.0">public_lib</build_depend>
  <depend version_eq="2.0.0">public_lib</depend>
</package>
""")
            with self.assertRaises(ValueError):
                XML.parse_package_xml(conflict)
        self.assertEqual(package.version_specs["public_lib"], ("gte", "1.2.0"))
        self.assertEqual(package.version_specs["runtime_lib"], ("eq", "2.0"))
        self.assertIn("public_lib;gte;1.2.0", text)

    def test_condition_uses_the_environment(self):
        os.environ["AUTOCMAKE_CONDITION_TEST"] = "on"
        with tempfile.TemporaryDirectory() as directory:
            path = write_package(directory, """<?xml version="1.0"?>
<package format="3">
  <name>sample</name>
  <version>1.2.3</version>
  <description>conditions</description>
  <maintainer email="dev@example.com">autocmake</maintainer>
  <license>Apache-2.0</license>
  <depend condition="$AUTOCMAKE_CONDITION_TEST == 'on'">kept</depend>
  <depend condition="$AUTOCMAKE_CONDITION_MISSING == 1">dropped</depend>
  <depend condition="not ($AUTOCMAKE_CONDITION_TEST != 'on')">also_kept</depend>
</package>
""")
            package = XML.parse_package_xml(path)
        self.assertEqual(package.build_depends, ["kept", "also_kept"])

    def test_invalid_condition_version_and_group(self):
        with tempfile.TemporaryDirectory() as directory:
            broken = write_package(directory, """<?xml version="1.0"?>
<package format="3">
  <name>sample</name>
  <version>1.2.3</version>
  <description>bad</description>
  <maintainer email="dev@example.com">autocmake</maintainer>
  <license>Apache-2.0</license>
  <depend condition="$">bad</depend>
</package>
""")
            with self.assertRaises(ValueError):
                XML.parse_package_xml(broken)
            version = write_package(directory, """<?xml version="1.0"?>
<package format="3">
  <name>sample</name>
  <version>1.2</version>
  <description>bad</description>
  <maintainer email="dev@example.com">autocmake</maintainer>
  <license>Apache-2.0</license>
</package>
""")
            with self.assertRaises(ValueError):
                XML.parse_package_xml(version)
            group = write_package(directory, """<?xml version="1.0"?>
<package format="3">
  <name>sample</name>
  <version>1.2.3</version>
  <description>bad</description>
  <maintainer email="dev@example.com">autocmake</maintainer>
  <license>Apache-2.0</license>
  <group_depend>rosidl_generator_packages</group_depend>
</package>
""")
            with self.assertRaises(ValueError):
                XML.parse_package_xml(group)


if __name__ == "__main__":
    unittest.main(argv=[sys.argv[0]])
