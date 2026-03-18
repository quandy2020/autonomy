#!/usr/bin/env python3
#
# Copyright 2025 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Generate MessageTypes.hh from .pb_index files (one header that includes all
# generated message headers). Compatible with gz-msgs tools layout; automsgs
# does not provide a Factory, so no register.cc is generated.

import argparse
import glob
import os
import sys


CC_HEADER = """/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

/* This file was automatically generated. Do not edit. */

#ifndef AUTOMSGS_MSGS_MESSAGE_TYPES_HH_
#define AUTOMSGS_MSGS_MESSAGE_TYPES_HH_
{includes}

#endif  // AUTOMSGS_MSGS_MESSAGE_TYPES_HH_
"""


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(
        description='Generate MessageTypes.hh from .pb_index files',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '--output-cpp-path',
        required=True,
        help='Base path of generated C++ files (contains .pb_index files)')
    parser.add_argument(
        '--output-header',
        help='Path of MessageTypes.hh to write (default: output-cpp-path/automsgs/msgs/MessageTypes.hh)')
    args = parser.parse_args(argv)

    output_cpp_path = os.path.normpath(args.output_cpp_path)
    includes = []

    # Scan for all .pb.h under output_cpp_path (skip details/ subdirs) to get correct include paths
    pattern = os.path.join(output_cpp_path, '**', '*.pb.h')
    for pb_h in sorted(glob.iglob(pattern, recursive=True)):
        if os.path.sep + 'details' + os.path.sep in pb_h or pb_h.endswith(os.path.sep + 'details'):
            continue
        rel = os.path.relpath(pb_h, output_cpp_path)
        include_path = rel.replace(os.path.sep, '/')
        includes.append('#include <' + include_path + '>')

    if not includes:
        # No .pb_index found; write a minimal header
        includes = ['/* No message types generated */']

    out_header = args.output_header
    if not out_header:
        out_header = os.path.join(output_cpp_path, 'automsgs', 'msgs', 'MessageTypes.hh')
    out_dir = os.path.dirname(out_header)
    os.makedirs(out_dir, exist_ok=True)
    with open(out_header, 'w') as f:
        f.write(CC_HEADER.format(includes='\n'.join(includes)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
