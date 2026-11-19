#!/usr/bin/env python3
"""Create compact GLB display meshes from imported robot assets.

Requires trimesh, fast-simplification, pycollada, scipy and pillow in the preparation
Python environment. These are not runtime dependencies. Google assets are copied
unchanged; Unitree STL/DAE geometry is converted with recorded provenance.
"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET


def prepare(source: Path, destination: Path, max_faces: int=5000):
    import trimesh
    cache={}
    for folder in sorted(source.iterdir()):
        if not folder.is_dir() or not folder.name.startswith(('unitree_','google_robot')):
            continue
        target=destination/folder.name
        if target.exists():raise FileExistsError(target)
        if folder.name=='google_robot':
            shutil.copytree(folder,target)
            continue
        target.mkdir(parents=True)
        urdf=folder/f'{folder.name}.urdf'
        root=ET.parse(urdf).getroot()
        provenance=json.loads((folder/'source.json').read_text())
        files={}
        for node in root.findall('.//mesh'):
            rel=node.attrib['filename']
            src=folder/rel
            converted=rel+'.glb' if src.suffix.lower() in ('.stl','.dae') else rel
            dest=target/converted
            if converted not in files:
                record=dict(provenance['files'][rel])
                dest.parent.mkdir(parents=True,exist_ok=True)
                if converted != rel:
                    fingerprint=hashlib.sha256(src.read_bytes()).hexdigest()
                    record['upstream_sha256']=fingerprint
                    cache_key=(fingerprint,src.suffix.lower())
                    if cache_key not in cache:
                        scene=trimesh.load(src,force='scene')
                        total_before,total_after=0,0
                        for key,mesh in list(scene.geometry.items()):
                            if not isinstance(mesh,trimesh.Trimesh):continue
                            total_before+=len(mesh.faces)
                            if len(mesh.faces)>max_faces:
                                simplified=mesh.simplify_quadric_decimation(face_count=max_faces)
                                # URDF solid materials remain in the URDF; retain
                                # the original dominant vertex/face colour too.
                                if mesh.visual.kind=='vertex':
                                    simplified.visual.vertex_colors=mesh.visual.vertex_colors.mean(axis=0).astype('uint8')
                                elif mesh.visual.kind=='face':
                                    simplified.visual.face_colors=mesh.visual.face_colors.mean(axis=0).astype('uint8')
                                elif mesh.visual.kind=='texture':
                                    from scipy.spatial import cKDTree

                                    uv=None
                                    if mesh.visual.uv is not None:
                                        nearest=cKDTree(mesh.vertices).query(simplified.vertices)[1]
                                        uv=mesh.visual.uv[nearest]
                                    simplified.visual=trimesh.visual.texture.TextureVisuals(
                                        uv=uv,material=mesh.visual.material.copy())
                                scene.geometry[key]=simplified
                                mesh=simplified
                            total_after+=len(mesh.faces)
                        cache[cache_key]=(scene.export(file_type='glb'),total_before,total_after)
                    data,before,after=cache[cache_key]
                    dest.write_bytes(data)
                    record.update(original_faces=before,display_faces=after,format='glb')
                else:
                    shutil.copy2(src,dest)
                record['sha256']=hashlib.sha256(dest.read_bytes()).hexdigest()
                files[converted]=record
            node.set('filename',converted)
        for name in ('LICENSE','README.md'):
            if (folder/name).is_file():
                shutil.copy2(folder/name,target/name)
                if name in provenance['files']:
                    files[name]=dict(provenance['files'][name])
        ET.indent(root)
        output=target/urdf.name
        ET.ElementTree(root).write(output,encoding='utf-8',xml_declaration=True)
        provenance['files']=files
        provenance['normalized_urdf_sha256']=hashlib.sha256(output.read_bytes()).hexdigest()
        provenance['changes'].append(f'Unitree STL/DAE converted to GLB and limited to {max_faces} faces per geometry; texture UVs sampled from nearest original vertices')
        (target/'source.json').write_text(json.dumps(provenance,indent=2)+'\n')
        (target/'README.md').write_text((target/'README.md').read_text()+
            f'\nHabitat display meshes: GLB, up to {max_faces} faces per geometry. Original URDF kinematics and scene transforms are retained. Texture UVs use nearest original vertices. Source hashes and face counts are in source.json.\n')
        print(f'{folder.name}: {sum(p.stat().st_size for p in target.rglob("*") if p.is_file())/1e6:.2f} MB',flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--destination',type=Path,required=True)
    parser.add_argument('--max-faces',type=int,default=5000)
    args=parser.parse_args()
    if args.max_faces<100:parser.error('--max-faces must be at least 100')
    prepare(args.source,args.destination,args.max_faces)
