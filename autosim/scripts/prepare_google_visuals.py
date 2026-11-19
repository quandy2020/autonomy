#!/usr/bin/env python3
"""Build Habitat-ready Google Robot visuals from the upstream ManiSkill assets.

Produces opaque GLBs with the reference palette (grey-white shells, gold finger
hardware, dark tyres) plus a modelled head sensor panel carrying the five
camera lenses and two status LEDs seen in the Everyday Robots photo set.

Habitat-Sim 0.3.3 samples glTF base-colour textures without an sRGB decode, so
the palette is baked straight into the atlas and every material is flagged
KHR_materials_unlit; that keeps the shells neutral instead of picking up the
yellow cast of the default four-light rig.

The generator needs trimesh and Pillow and runs outside Habitat:

    python scripts/prepare_google_visuals.py --source /path/to/ManiSkill-GoogleRobot

--source points at an upstream checkout containing meshes/3obj and
meshes/coarse_meshes.  Outputs land in robot/google_robot/meshes/habitat and
robot/google_robot/appearance.json.
"""
from __future__ import annotations

import argparse
import json
import math
import struct
from pathlib import Path

import numpy as np
import trimesh
from PIL import Image

SHELL = (236, 236, 236)
GOLD = (219, 176, 61)
TYRE = (26, 26, 28)


def recolour(image: Image.Image, *, gold: bool = False, tyre: bool = False) -> Image.Image:
    """Map the upstream grey/gold palette onto the reference palette."""
    pixels = np.array(image.convert('RGBA'), copy=True)
    rgb = pixels[..., :3].astype(np.float32)
    if tyre:
        rgb[:] = TYRE
    elif gold:
        rgb[:] = GOLD
    else:
        # Neutral shell pixels (the upstream 222 grey) become the reference
        # off-white; dark lettering, vents and thin rings keep their detail.
        spread = rgb.max(axis=2) - rgb.min(axis=2)
        neutral = (spread < 14) & (rgb.min(axis=2) > 150)
        scale = np.array(SHELL, np.float32) / 222.0
        rgb[neutral] = np.clip(rgb[neutral] * scale, 0, 255)
        # The torso carries a gold service panel that is white on the real robot.
        goldish = (rgb[..., 0] > 150) & (rgb[..., 1] > 90) & (rgb[..., 0] > 1.35 * rgb[..., 2])
        rgb[goldish] = SHELL
    pixels[..., :3] = np.clip(rgb, 0, 255).astype(np.uint8)
    pixels[..., 3] = 255
    return Image.fromarray(pixels)


def load_visual(path: Path) -> trimesh.Scene:
    return trimesh.load(path, force='scene')


def material_for(name: str, image: Image.Image, *, gold: bool, tyre: bool,
                 emissive: Image.Image | None = None) -> trimesh.visual.material.PBRMaterial:
    return trimesh.visual.material.PBRMaterial(
        name=name,
        baseColorTexture=recolour(image, gold=gold, tyre=tyre),
        baseColorFactor=[1.0, 1.0, 1.0, 1.0],
        metallicFactor=0.0 if not tyre else 0.15,
        roughnessFactor=0.62 if not tyre else 0.85,
        emissiveTexture=emissive,
        emissiveFactor=[1.0, 1.0, 1.0] if emissive is not None else [0.0, 0.0, 0.0],
        alphaMode='OPAQUE',
        doubleSided=True,
    )


def mark_unlit(path: Path) -> None:
    """Flag every material unlit so Habitat renders the baked palette exactly."""
    data = path.read_bytes()
    length = struct.unpack_from('<I', data, 12)[0]
    document = json.loads(data[20:20 + length])
    for material in document.get('materials', []):
        material.setdefault('extensions', {})['KHR_materials_unlit'] = {}
    document['extensionsUsed'] = list(dict.fromkeys(
        document.get('extensionsUsed', []) + ['KHR_materials_unlit']))
    encoded = json.dumps(document, separators=(',', ':')).encode()
    encoded += b' ' * (-len(encoded) % 4)
    binary = data[20 + length:]
    path.write_bytes(struct.pack('<4sII', b'glTF', 2, 20 + len(encoded) + len(binary))
                     + struct.pack('<I4s', len(encoded), b'JSON') + encoded + binary)


def convert(source: Path, relative: str, destination: Path, *, gold: bool, tyre: bool,
            emissive: Path | None) -> int:
    scene = load_visual(source / relative)
    glow = Image.open(emissive).convert('RGB') if emissive is not None else None
    faces = 0
    for key, mesh in list(scene.geometry.items()):
        if not isinstance(mesh, trimesh.Trimesh):
            continue
        uv = np.array(mesh.visual.uv, copy=True)
        previous = mesh.visual.material
        image = getattr(previous, 'baseColorTexture', None) or getattr(previous, 'image', None)
        if image is None:
            raise ValueError(f'{relative}: no diffuse texture')
        mesh.visual = trimesh.visual.TextureVisuals(
            uv=uv, material=material_for(f'google_{key}', image, gold=gold, tyre=tyre, emissive=glow))
        faces += len(mesh.faces)
    out = destination / (Path(relative).name + '.glb')
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_bytes(scene.export(file_type='glb'))
    mark_unlit(out)
    return faces


def ellipse_ring(a: float, b: float, x: float, segments: int = 96) -> np.ndarray:
    t = np.linspace(0.0, 2.0 * math.pi, segments, endpoint=False)
    return np.stack([np.full_like(t, x), a * np.cos(t), b * np.sin(t)], axis=1)


def disc(a: float, b: float, x: float, centre=(0.0, 0.0), segments: int = 96) -> trimesh.Trimesh:
    rim = ellipse_ring(a, b, x, segments)
    rim[:, 1] += centre[0]
    rim[:, 2] += centre[1]
    vertices = np.vstack([[[x, centre[0], centre[1]]], rim])
    faces = [[0, i + 1, (i + 1) % segments + 1] for i in range(segments)]
    return trimesh.Trimesh(vertices=vertices, faces=np.array(faces), process=False)


def annulus(inner: tuple[float, float], outer: tuple[float, float], x: float,
            segments: int = 96) -> trimesh.Trimesh:
    i = ellipse_ring(*inner, x, segments)
    o = ellipse_ring(*outer, x, segments)
    vertices = np.vstack([i, o])
    faces = []
    for k in range(segments):
        n = (k + 1) % segments
        # Wind counter-clockwise seen from +x so the ring faces the camera.
        faces.append([k, segments + n, n])
        faces.append([k, segments + k, segments + n])
    return trimesh.Trimesh(vertices=vertices, faces=np.array(faces), process=False)


def tube(radius: float, x0: float, x1: float, centre=(0.0, 0.0), segments: int = 48) -> trimesh.Trimesh:
    t = np.linspace(0.0, 2.0 * math.pi, segments, endpoint=False)
    ring = np.stack([radius * np.cos(t), radius * np.sin(t)], axis=1)
    back = np.column_stack([np.full(segments, x0), ring[:, 0] + centre[0], ring[:, 1] + centre[1]])
    front = np.column_stack([np.full(segments, x1), ring[:, 0] + centre[0], ring[:, 1] + centre[1]])
    vertices = np.vstack([back, front])
    faces = []
    for k in range(segments):
        n = (k + 1) % segments
        faces.append([k, n, segments + n])
        faces.append([k, segments + n, segments + k])
    return trimesh.Trimesh(vertices=vertices, faces=np.array(faces), process=False)


def head_panel() -> trimesh.Scene:
    """Five-lens sensor head matching the CNET close-up.

    Every part is merged into one mesh whose colours live in a small atlas, so
    the panel goes through the same textured path as the shell meshes and keeps
    the reference colours in Habitat.
    """
    front = 0.1192
    palette = {
        'panel': (12, 12, 14),
        'bezel': (58, 58, 61),
        'lens_bezel': (74, 74, 77),
        'lens_rim': (150, 150, 152),
        'lens_chrome': (206, 206, 209),
        'lens_glass': (26, 30, 38),
        'lens_iris': (10, 12, 18),
        'led': (248, 250, 255),
    }
    names = list(palette)
    cell, columns = 64, 4
    rows = (len(names) + columns - 1) // columns
    atlas = Image.new('RGB', (cell * columns, cell * rows), (0, 0, 0))
    centres = {}
    for index, name in enumerate(names):
        column, row = index % columns, index // columns
        atlas.paste(Image.new('RGB', (cell, cell), palette[name]), (column * cell, row * cell))
        centres[name] = ((column + 0.5) / columns, 1.0 - (row + 0.5) / rows)

    parts: list[tuple[str, trimesh.Trimesh]] = []

    def add(name: str, mesh: trimesh.Trimesh) -> None:
        parts.append((name, mesh))

    add('panel', disc(0.088, 0.031, front + 0.0006))
    add('bezel', annulus((0.088, 0.031), (0.0915, 0.0338), front + 0.0009))

    lenses = [(-0.062, 0.0112), (-0.031, 0.0104), (0.0, 0.0118), (0.031, 0.0104), (0.062, 0.0112)]
    for index, (y, radius) in enumerate(lenses):
        centre = (y, 0.004)
        add('lens_bezel', tube(radius * 1.16, front + 0.0006, front + 0.0026, centre))
        if index == 2:
            add('lens_chrome',
                annulus((radius * 1.02, radius * 1.02), (radius * 1.42, radius * 1.42),
                        front + 0.0032, segments=64))
        else:
            add('lens_rim',
                annulus((radius * 1.02, radius * 1.02), (radius * 1.18, radius * 1.18),
                        front + 0.0032, segments=48))
        add('lens_glass', disc(radius, radius, front + 0.0036, centre, segments=48))
        add('lens_iris', disc(radius * 0.42, radius * 0.42, front + 0.0040, centre, segments=32))
    for y in (-0.046, 0.046):
        add('led', disc(0.0028, 0.0028, front + 0.0012, (y, 0.021), segments=24))

    vertices, faces, uvs = [], [], []
    offset = 0
    for name, mesh in parts:
        vertices.append(np.asarray(mesh.vertices, dtype=np.float64))
        faces.append(np.asarray(mesh.faces, dtype=np.int64) + offset)
        uvs.append(np.tile(centres[name], (len(mesh.vertices), 1)))
        offset += len(mesh.vertices)
    merged = trimesh.Trimesh(vertices=np.vstack(vertices), faces=np.vstack(faces), process=False)
    material = trimesh.visual.material.PBRMaterial(
        name='google_head_panel', baseColorTexture=atlas, baseColorFactor=[1.0, 1.0, 1.0, 1.0],
        metallicFactor=0.0, roughnessFactor=0.45, alphaMode='OPAQUE', doubleSided=True)
    merged.visual = trimesh.visual.TextureVisuals(uv=np.vstack(uvs), material=material)
    scene = trimesh.Scene()
    scene.add_geometry(merged, geom_name='head_sensor_panel', node_name='head_sensor_panel')
    return scene


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', required=True, type=Path)
    root = Path(__file__).resolve().parents[1]
    parser.add_argument('--destination', type=Path,
                        default=root / 'robot/google_robot/meshes/habitat')
    parser.add_argument('--appearance', type=Path,
                        default=root / 'robot/google_robot/appearance.json')
    args = parser.parse_args()

    source, destination = args.source, args.destination
    destination.mkdir(parents=True, exist_ok=True)
    visuals = {
        'meshes/3obj/link_base.glb': dict(gold=False, tyre=False),
        'meshes/3obj/link_bicep.3obj.obj': dict(gold=False, tyre=False),
        'meshes/3obj/link_elbow.3obj.obj': dict(gold=False, tyre=False),
        'meshes/3obj/link_forearm.3obj.obj': dict(gold=False, tyre=False),
        'meshes/3obj/link_gripper.3obj.obj': dict(gold=False, tyre=False),
        'meshes/3obj/link_head_pan.3obj.obj': dict(gold=False, tyre=False),
        'meshes/3obj/link_head_tilt.3obj.obj': dict(gold=False, tyre=False, emissive='meshes/3obj/meta_emissive.png'),
        'meshes/3obj/link_shoulder.3obj.obj': dict(gold=False, tyre=False),
        'meshes/3obj/link_torso.3obj.obj': dict(gold=False, tyre=False),
        'meshes/3obj/link_wrist.3obj.obj': dict(gold=False, tyre=False),
        'meshes/3obj/link_finger_base_simplified.3obj.obj': dict(gold=True, tyre=False),
        'meshes/3obj/link_finger_tip_simplified.3obj.obj': dict(gold=True, tyre=False),
        'meshes/3obj/link_finger_nail.3obj.obj': dict(gold=True, tyre=False),
        'meshes/3obj/link_wheel.3obj.obj': dict(gold=False, tyre=True),
        'meshes/coarse_meshes/link_rear_wheel.glb': dict(gold=False, tyre=True),
    }
    report = {}
    for relative, options in visuals.items():
        emissive = source / options['emissive'] if options.get('emissive') else None
        faces = convert(source, relative, destination, gold=options['gold'],
                        tyre=options['tyre'], emissive=emissive)
        report[Path(relative).name + '.glb'] = faces
        print(f'{Path(relative).name}.glb  {faces} triangles')

    panel = head_panel()
    panel_path = destination / 'head_sensor_panel.glb'
    panel_path.write_bytes(panel.export(file_type='glb'))
    mark_unlit(panel_path)
    report['head_sensor_panel.glb'] = int(sum(len(g.faces) for g in panel.geometry.values()))
    print(f'head_sensor_panel.glb  {report["head_sensor_panel.glb"]} triangles')
    args.appearance.parent.mkdir(parents=True, exist_ok=True)
    args.appearance.write_text(json.dumps({
        'reference': 'Everyday Robots photo set (CNET kitchen tour)',
        'palette_srgb': {
            'shell': list(SHELL), 'finger_gold': list(GOLD), 'tyre': list(TYRE),
            'head_panel': {
                'plate': [12, 12, 14], 'bezel': [58, 58, 61],
                'lens_bezel': [74, 74, 77], 'lens_rim': [150, 150, 152],
                'lens_chrome': [206, 206, 209], 'lens_glass': [26, 30, 38],
                'lens_iris': [10, 12, 18], 'led': [248, 250, 255],
            },
        },
        'materials': {
            'extension': 'KHR_materials_unlit',
            'reason': 'Habitat-Sim 0.3.3 samples base-colour textures without sRGB '
                      'decoding and tints lit shells yellow under its default rig',
            'alpha_mode': 'OPAQUE',
        },
        'head_panel': {
            'file': 'meshes/habitat/head_sensor_panel.glb',
            'description': 'black face plate, dark bezel, five lenses with silver rims '
                           '(chrome ring on the centre lens), two status LEDs',
            'lens_centres_m': [-0.062, -0.031, 0.0, 0.031, 0.062],
        },
        'triangles': report,
    }, indent=2) + '\n')


if __name__ == '__main__':
    main()
