#!/usr/bin/env python3
#
# Copyright 2025 The Openbot Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Generate C++/Python from .proto using standard protoc (no custom plugin).
# Optionally copies .pb.h to details/ and writes .pb_index for factory use.

import argparse
import os
import shutil
import subprocess
import sys


def main(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser(
        description='Generate protobuf C++/Python support files (no plugin)',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--protoc-exec', required=True, help='Path to protoc')
    parser.add_argument('--generator-bin', nargs='?', default='', dest='generator_bin', help='Unused, for compat')
    parser.add_argument('--proto-path', required=True, help='Proto root ( -I )', action='append')
    parser.add_argument('--input-path', required=True, help='Input .proto path (abs or rel to proto-path)')
    parser.add_argument('--output-cpp-path', help='Output dir for C++')
    parser.add_argument('--output-python-path', help='Output dir for Python')
    parser.add_argument('--generate-cpp', action='store_true')
    parser.add_argument('--generate-python', action='store_true')
    parser.add_argument('--dependency-proto-descs', nargs='*', help='Descriptor set deps')
    parser.add_argument('--dllexport-decl', help='DLL export macro')
    args = parser.parse_args(argv)

    proto_path = args.proto_path[0] if args.proto_path else ''
    if not os.path.isabs(args.input_path):
        input_abs = os.path.join(proto_path, args.input_path)
    else:
        input_abs = args.input_path

    if not os.path.isfile(input_abs):
        print(f'Input proto not found: {input_abs}', file=sys.stderr)
        sys.exit(1)

    cmd = [args.protoc_exec]
    for p in args.proto_path:
        cmd += [f'--proto_path={p}']
    if args.dependency_proto_descs:
        for d in args.dependency_proto_descs:
            cmd += [f'--descriptor_set_in={d}']

    if args.generate_cpp and args.output_cpp_path:
        os.makedirs(args.output_cpp_path, exist_ok=True)
        cpp_out = args.output_cpp_path
        if args.dllexport_decl:
            cpp_out = f'dllexport_decl={args.dllexport_decl}:{cpp_out}'
        cmd += [f'--cpp_out={cpp_out}']
    if args.generate_python and args.output_python_path:
        os.makedirs(args.output_python_path, exist_ok=True)
        cmd += [f'--python_out={args.output_python_path}']

    cmd += [input_abs]

    try:
        subprocess.check_call(cmd)
    except subprocess.CalledProcessError as e:
        print(f'protoc failed: {e}', file=sys.stderr)
        sys.exit(1)

    # Place .pb.h into details/ and write .pb_index (for gz-msgs-style layout/factory)
    if args.generate_cpp and args.output_cpp_path:
        # input_path like automsgs/msgs/geometry_msgs/Pose.proto -> rel_dir = automsgs/msgs/geometry_msgs
        rel = args.input_path.replace('\\', '/')
        if rel.endswith('.proto'):
            rel = rel[:-len('.proto')]
        base = os.path.basename(rel)
        rel_dir = os.path.dirname(rel)
        out_header = os.path.join(args.output_cpp_path, rel_dir, base + '.pb.h')
        out_detail_dir = os.path.join(args.output_cpp_path, rel_dir, 'details')
        out_detail_header = os.path.join(out_detail_dir, base + '.pb.h')
        if os.path.isfile(out_header):
            os.makedirs(out_detail_dir, exist_ok=True)
            shutil.copy2(out_header, out_detail_header)

        # .pb_index: one line = message name (filename without extension)
        unique_name = rel.replace('/', '_').replace('\\', '_')
        index_path = os.path.join(args.output_cpp_path, unique_name + '.pb_index')
        with open(index_path, 'w') as f:
            f.write(os.path.splitext(base)[0] + '\n')

    return 0


if __name__ == '__main__':
    sys.exit(main())
