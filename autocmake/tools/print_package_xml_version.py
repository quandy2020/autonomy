#!/usr/bin/env python3
# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0

"""Print the MAJOR.MINOR.PATCH version from a package.xml file."""

import re
import sys
import xml.etree.ElementTree as ET


def main(argv):
    if len(argv) != 1:
        print("usage: print_package_xml_version.py PACKAGE_XML", file=sys.stderr)
        return 2
    try:
        root = ET.parse(argv[0]).getroot()
    except ET.ParseError as exc:
        print(f"{argv[0]}: {exc}", file=sys.stderr)
        return 1
    if root.tag != "package":
        print(f"{argv[0]}: root element is '{root.tag}', expected 'package'", file=sys.stderr)
        return 1
    version = root.find("version")
    text = "" if version is None or version.text is None else version.text.strip()
    if not re.match(r"^\d+\.\d+\.\d+$", text):
        print(f"{argv[0]}: version '{text}' is not MAJOR.MINOR.PATCH", file=sys.stderr)
        return 1
    print(text, end="")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
