#!/usr/bin/env python3
"""
Rename all .proto files to snake_case (lowercase + underscores) and update
import paths in proto files. Run from repo root or from proto/.
"""
import os
import re
import glob

def to_snake(name):
    """PascalCase/CamelCase -> snake_case. E.g. PointField -> point_field."""
    s = []
    for i, c in enumerate(name):
        if c.isupper() and i > 0:
            s.append('_')
        s.append(c.lower())
    return ''.join(s)

def main():
    base = os.path.dirname(os.path.abspath(__file__))
    os.chdir(base)
    # Collect all .proto files
    protos = []
    for root, _, _ in os.walk('.'):
        for f in glob.glob(os.path.join(root, '*.proto')):
            protos.append(f.replace(os.sep, '/').lstrip('./'))
    # Build mapping: "Header.proto" -> "header.proto", "PointField.proto" -> "point_field.proto"
    name_map = {}
    for path in protos:
        basename = os.path.basename(path)
        stem, ext = os.path.splitext(basename)
        snake_stem = to_snake(stem)
        new_basename = snake_stem + ext
        if basename != new_basename:
            name_map[basename] = new_basename
    # Also map for import replacement: we need "Header.proto" -> "header.proto" in import lines
    # Rename files (from deepest paths first to avoid dir renames)
    protos_sorted = sorted(protos, key=lambda p: (-p.count('/'), p))
    for path in protos_sorted:
        dirname = os.path.dirname(path)
        basename = os.path.basename(path)
        new_basename = name_map.get(basename, basename)
        if basename == new_basename:
            continue
        new_path = os.path.join(dirname, new_basename).replace(os.sep, '/') if dirname else new_basename
        old_full = os.path.join(base, path)
        new_full = os.path.join(base, new_path)
        if os.path.exists(old_full):
            os.rename(old_full, new_full)
            print('Rename:', path, '->', new_path)
    # Update imports in all .proto files (f is already relative path from glob)
    # Replace any import "path/Name.proto" with "path/name_snake.proto"
    import_pat = re.compile(r'import\s+"([^"]+)/([^/"]+\.proto)"')
    def replace_import(m):
        path_prefix, filename = m.group(1), m.group(2)
        stem, ext = os.path.splitext(filename)
        return 'import "' + path_prefix + '/' + to_snake(stem) + ext + '"'
    for root, _, _ in os.walk('.'):
        for f in glob.glob(os.path.join(root, '*.proto')):
            path = os.path.join(base, f)
            with open(path, 'r', encoding='utf-8') as fp:
                content = fp.read()
            new_content = import_pat.sub(replace_import, content)
            if new_content != content:
                with open(path, 'w', encoding='utf-8') as fp:
                    fp.write(new_content)
                print('Updated imports in:', path)
    print('Done.')

if __name__ == '__main__':
    main()
