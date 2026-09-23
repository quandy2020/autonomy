#!/usr/bin/env python3
# Copyright 2026 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0

"""Fail when a test did not write a result file.

CTest runs this after a gtest. A crash or timeout leaves no xml. This writes
a failure report and exits non-zero so the gap is visible to CTest.
"""

import os
import sys


def main(argv):
    if len(argv) != 1:
        print("usage: check_test_ran.py RESULT.xml", file=sys.stderr)
        return 2
    result = argv[0]
    if os.path.isfile(result) and os.path.getsize(result) > 0:
        print(f"test results found: {result}")
        return 0

    directory = os.path.dirname(result)
    if directory:
        os.makedirs(directory, exist_ok=True)
    name = os.path.basename(result)
    classname = name[:-4] if name.endswith(".xml") else name
    print(f"test did not write results: {result}", file=sys.stderr)
    with open(result, "w", encoding="utf-8") as handle:
        handle.write(
            "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
            f"<testsuite tests=\"1\" failures=\"1\" time=\"0\" errors=\"0\" name=\"{name}\">\n"
            f"  <testcase name=\"test_ran\" status=\"run\" time=\"0\" classname=\"{classname}\">\n"
            f"    <failure message=\"No results in {result}. The test did not run to completion.\" type=\"\"/>\n"
            "  </testcase>\n"
            "</testsuite>\n"
        )
    return 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
