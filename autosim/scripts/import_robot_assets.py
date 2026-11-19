#!/usr/bin/env python3
"""Import pinned upstream URDFs and their mesh dependencies without ROS/xacro.

Run from any directory; downloads are cached. Existing robot folders are refused.
"""
from __future__ import annotations
import argparse
import concurrent.futures
import hashlib
import json
import posixpath
from pathlib import Path
import shutil
import urllib.request
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
SOURCES = {
    'google': ('haosulab/ManiSkill-GoogleRobot', '3aa86c751ba508acb1272f73dbbd827cd25337b0'),
    'unitree': ('unitreerobotics/unitree_ros', '157ff950351e5f535e03c5d85a16b7d594b3dbcc'),
}
# Each name is one independent robot asset package, with its original meshes.
UNITREE = {
 'g1': 'g1_description/g1_29dof.urdf',
 'go2': 'go2_description/urdf/go2_description.urdf',
}


def fetch(url: str, target: Path) -> Path:
    if target.is_file():
        return target
    target.parent.mkdir(parents=True, exist_ok=True)
    error = None
    for _ in range(3):
        try:
            request = urllib.request.Request(url, headers={'User-Agent': 'autosim-robot-assets'})
            with urllib.request.urlopen(request, timeout=90) as response:
                data = response.read()
            temp = target.with_suffix(target.suffix + '.part')
            temp.write_bytes(data)
            temp.replace(target)
            return target
        except Exception as exc:
            error = exc
    raise RuntimeError(f'Cannot download {url}: {error}')


def import_assets(cache: Path, destination: Path, names: list[str] | None = None) -> None:
    manifests = {}
    for source, (repo, revision) in SOURCES.items():
        tree = fetch(f'https://api.github.com/repos/{repo}/git/trees/{revision}?recursive=1', cache/source/'tree.json')
        data = json.loads(tree.read_text())
        if data.get('truncated'):
            raise RuntimeError(f'{repo}: incomplete GitHub tree')
        manifests[source] = {item['path']: item for item in data['tree'] if item['type'] == 'blob'}
    models = {'google_robot': ('google', 'google_robot_meta_sim_fix_fingertip.urdf')}
    models.update({f'unitree_{name}': ('unitree', f'robots/{path}') for name, path in UNITREE.items()})
    if names:
        unknown = set(names)-set(models)
        if unknown:
            raise ValueError(f'unknown models: {sorted(unknown)}')
        models = {name: models[name] for name in names}
    plans = []
    downloads = set()
    for name, (source, urdf) in models.items():
        target = destination/name
        if target.exists():
            raise FileExistsError(f'{target} exists; use a fresh destination')
        repo, rev = SOURCES[source]
        source_path = fetch(f'https://raw.githubusercontent.com/{repo}/{rev}/{urdf}', cache/source/urdf)
        original = source_path.read_bytes()
        blob = hashlib.sha1(b'blob '+str(len(original)).encode()+b'\0'+original).hexdigest()
        if blob != manifests[source][urdf]['sha']:
            raise RuntimeError(f'upstream URDF blob mismatch: {urdf}')
        root = ET.parse(source_path).getroot()
        changes = ['mesh paths made relative to this robot folder']
        for index, material in enumerate(root.findall('.//material')):
            if material.get('name') is None:
                material.set('name', f'autosim_material_{index}')
                changes.append(f'unnamed material assigned autosim_material_{index}; colour preserved')
        child_links = {j.find('child').get('link') for j in root.findall('joint')}
        for joint in root.findall('joint'):
            if joint.get('type') == 'floating' and joint.find('parent').get('link') not in child_links:
                joint.set('type', 'fixed')
                changes.append(f"floating root joint {joint.get('name')} fixed; base pose controlled by the simulator")
        files = {}
        package = urdf.split('/')[1] if source == 'unitree' else ''
        package_root = f'robots/{package}' if package else ''
        available = manifests[source]
        for node in root.findall('.//mesh'):
            raw = node.attrib['filename']
            if raw.startswith('package://'):
                package_rel = raw[len('package://'):]
                candidate = 'robots/' + package_rel
            else:
                candidate = posixpath.normpath(posixpath.join(posixpath.dirname(urdf), raw))
            if candidate not in available:
                candidates = [key for key in available if key.startswith(package_root+'/') and key.lower().endswith('/'+raw.split('/')[-1].lower())]
                if len(candidates) != 1:
                    raise FileNotFoundError(f'{name}: unresolved mesh {raw}, candidates={candidates}')
                candidate = candidates[0]
            rel = candidate[len(package_root)+1:] if package_root else candidate
            if rel.startswith('meshes/'):
                rel = rel[len('meshes/'):]
            output = 'meshes/'+rel
            files[candidate] = output
            node.set('filename', output)
        # OBJ material libraries and textures must retain their relative locations.
        mesh_dirs = {posixpath.dirname(key) for key in files}
        for key in available:
            if posixpath.dirname(key) in mesh_dirs and Path(key).suffix.lower() in ('.mtl','.png','.jpg','.jpeg','.tga','.bmp'):
                rel = key[len(package_root)+1:] if package_root else key
                if rel.startswith('meshes/'):
                    rel = rel[len('meshes/'):]
                files[key] = 'meshes/'+rel
        if 'LICENSE' in available:
            files['LICENSE'] = 'LICENSE'
        downloads.update((source,key) for key in files)
        plans.append((name,source,urdf,root,files,changes))
    def download(item):
        source, key = item
        repo, rev = SOURCES[source]
        path = fetch(f'https://raw.githubusercontent.com/{repo}/{rev}/{key}', cache/source/key)
        blob = hashlib.sha1(b'blob '+str(path.stat().st_size).encode()+b'\0'+path.read_bytes()).hexdigest()
        if blob != manifests[source][key]['sha']:
            raise RuntimeError(f'upstream blob mismatch: {key}')
        return key
    with concurrent.futures.ThreadPoolExecutor(max_workers=12) as pool:
        for i,_ in enumerate(pool.map(download, sorted(downloads)),1):
            if i % 30 == 0:
                print(f'downloaded {i}/{len(downloads)}', flush=True)
    for name, source, urdf, root, files, changes in plans:
        target = destination/name
        target.mkdir(parents=True)
        for key,out in files.items():
            dest = target/out
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(cache/source/key, dest)
        ET.indent(root)
        ET.ElementTree(root).write(target/f'{name}.urdf', encoding='utf-8', xml_declaration=True)
        repo, rev = SOURCES[source]
        provenance = {'repository': f'https://github.com/{repo}', 'revision': rev, 'urdf': urdf,
                      'changes': changes,
                      'upstream_urdf_sha256': hashlib.sha256((cache/source/urdf).read_bytes()).hexdigest(),
                      'normalized_urdf_sha256': hashlib.sha256((target/f'{name}.urdf').read_bytes()).hexdigest(),
                      'files': {out: {'upstream': key, 'sha256': hashlib.sha256((target/out).read_bytes()).hexdigest()} for key,out in files.items()}}
        (target/'source.json').write_text(json.dumps(provenance, indent=2)+'\n')
        (target/'README.md').write_text(f'# {name}\n\nUpstream: https://github.com/{repo}\n\nRevision: `{rev}`\n\nURDF: `{urdf}`\n\nGeometry is copied from upstream. URDF compatibility changes are recorded in source.json.\n'+
            ('\nLicense: see LICENSE.\n' if 'LICENSE' in files else '\nThe upstream snapshot contains no LICENSE file; no new license is assigned to these assets.\n'))
        print(f'imported {name}', flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cache', type=Path, default=Path('/tmp/autosim-robot-downloads'))
    parser.add_argument('--destination', type=Path, default=ROOT/'robot')
    parser.add_argument('--models', nargs='+')
    args = parser.parse_args()
    import_assets(args.cache, args.destination, args.models)
