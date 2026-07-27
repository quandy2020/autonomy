#!/usr/bin/env python3
# Copyright 2026 The Openbot Authors (duyongquan)
#
# Convert MCAP recordings to Autolink .record (protobuf channels).
#
# Native conversion requires Autolink Python bindings + MCAP reader.
# This script is a placeholder until mcap→record ships in autolink tools.
#
# Usage:
#   mcap_to_record.py input.mcap output.record

from __future__ import annotations

import argparse
import sys


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Convert MCAP to Autolink .record (stub)."
    )
    parser.add_argument("input_mcap", help="Path to .mcap file")
    parser.add_argument("output_record", help="Path to output .record file")
    args = parser.parse_args()

    sys.stderr.write(
        "mcap_to_record: native MCAP→.record conversion is not yet implemented.\n"
        "Track Autolink developer tools for mcap_to_record.\n\n"
        "  input:  {0}\n"
        "  output: {1}\n".format(args.input_mcap, args.output_record)
    )
    return 1


if __name__ == "__main__":
    sys.exit(main())
