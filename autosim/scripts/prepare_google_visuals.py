#!/usr/bin/env python3
"""Build Habitat-ready Google Robot visuals from the upstream ManiSkill assets.

Produces opaque GLBs with the reference palette (grey-white shells, gold finger
hardware, dark tyres) plus a modelled head sensor panel carrying the five
camera lenses, green LED ring and two status LEDs seen in the Everyday Robots
photo set.  A small base trim adds the dark lower bumper, front status light
and ``040`` side decals from the same reference.

Habitat-Sim 0.3.3 samples glTF base-colour textures without an sRGB decode, so
the palette is baked straight into the atlas and every material is flagged
KHR_materials_unlit; that keeps the shells neutral instead of picking up the
yellow cast of the default four-light rig.

The generator needs trimesh and Pillow and runs outside Habitat.  ``--source``
is optional: omit it to regenerate only the modelled head panel and base trim
while keeping the already converted upstream GLBs:

    python scripts/prepare_google_visuals.py --source /path/to/ManiSkill-GoogleRobot
    python scripts/prepare_google_visuals.py

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
BUMPER = (28, 28, 30)
DECAL_INK = (76, 76, 79)
LED_RING = (42, 232, 92)


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


def mark_unlit(path: Path, skip: set[str] | None = None) -> None:
    """Flag every material unlit so Habitat renders the baked palette exactly."""
    skip = skip or set()
    data = path.read_bytes()
    length = struct.unpack_from('<I', data, 12)[0]
    document = json.loads(data[20:20 + length])
    for material in document.get('materials', []):
        if material.get('name') in skip:
            continue
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
        'led_ring': LED_RING,
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
    add('led_ring', annulus((0.0915, 0.0338), (0.0945, 0.0358),
                            front + 0.0014, segments=96))

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


def solid_mesh(mesh: trimesh.Trimesh, name: str, colour: tuple[int, int, int]) -> trimesh.Trimesh:
    """Attach a one-colour texture so Habitat's glTF path renders it reliably."""
    image = Image.new('RGB', (4, 4), colour)
    material = trimesh.visual.material.PBRMaterial(
        name=name,
        baseColorTexture=image,
        baseColorFactor=[1.0, 1.0, 1.0, 1.0],
        metallicFactor=0.0,
        roughnessFactor=0.8,
        alphaMode='OPAQUE',
        doubleSided=True,
    )
    mesh.visual = trimesh.visual.TextureVisuals(
        uv=np.zeros((len(mesh.vertices), 2)), material=material
    )
    return mesh


def box(extents: tuple[float, float, float], centre: tuple[float, float, float],
        name: str, colour: tuple[int, int, int]) -> trimesh.Trimesh:
    mesh = trimesh.creation.box(extents=extents)
    mesh.apply_translation(centre)
    return solid_mesh(mesh, name, colour)


def digit_ring(cx: float, cz: float, rx: float, rz: float,
               inner: float = 0.56, segments: int = 48) -> tuple[list, list]:
    """A flat elliptical ring used for the ``0`` glyphs."""
    t = np.linspace(0.0, 2.0 * math.pi, segments, endpoint=False)
    outer = np.column_stack([cx + rx * np.cos(t), cz + rz * np.sin(t)])
    inner_ring = np.column_stack([cx + rx * inner * np.cos(t),
                                  cz + rz * inner * np.sin(t)])
    points = np.vstack([outer, inner_ring]).tolist()
    faces = []
    for index in range(segments):
        nxt = (index + 1) % segments
        faces.append([index, nxt, segments + nxt])
        faces.append([index, segments + nxt, segments + index])
    return points, faces


def digit_quad(x0: float, z0: float, x1: float, z1: float,
               x2: float, z2: float, x3: float, z3: float) -> tuple[list, list]:
    """A flat quad used for the strokes of the ``4`` glyph."""
    return [[x0, z0], [x1, z1], [x2, z2], [x3, z3]], [[0, 1, 2], [0, 2, 3]]


def decal(side: float, name: str) -> trimesh.Trimesh:
    """Build ``040`` as thin geometry so Habitat needs no alpha test.

    Habitat-Sim 0.3.3 ignores glTF alpha modes on this asset path, so a
    textured plane would render its transparent pixels as a black rectangle.
    The glyph strokes follow the base's side slope and are only 0.5 mm proud.
    """
    parts: list[tuple[list, list]] = []
    for cx in (-0.280, -0.220):
        parts.append(digit_ring(cx, 0.050, 0.016, 0.022))
    for points, faces in (
        digit_quad(0.006, 0.028, 0.012, 0.028, 0.012, 0.072, 0.006, 0.072),
        digit_quad(0.006, 0.072, 0.000, 0.072, -0.014, 0.046, -0.008, 0.046),
        digit_quad(-0.018, 0.046, 0.012, 0.046, 0.012, 0.052, -0.018, 0.052),
    ):
        parts.append(([[x - 0.247, z] for x, z in points], faces))

    vertices, faces, offset = [], [], 0
    for points, part_faces in parts:
        vertices.extend(points)
        faces.extend([[a + offset, b + offset, c + offset] for a, b, c in part_faces])
        offset += len(points)
    points = np.asarray(vertices, dtype=np.float64)
    z = points[:, 1]
    y = side * (0.186936 - 0.08359 * (z - 0.03) + 0.0005)
    mesh = trimesh.Trimesh(vertices=np.column_stack([points[:, 0], y, z]),
                           faces=np.asarray(faces, dtype=np.int64), process=False)
    return solid_mesh(mesh, name, DECAL_INK)


def base_trim() -> trimesh.Scene:
    """Thin dark lower skirt, front status light and ``040`` side decals."""
    scene = trimesh.Scene()
    parts = {
        'bumper_front': box((0.004, 0.330, 0.014), (0.1365, 0.0, -0.029),
                            'bumper_front', BUMPER),
        'bumper_rear': box((0.004, 0.330, 0.014), (-0.4005, 0.0, -0.029),
                           'bumper_rear', BUMPER),
        'bumper_left': box((0.535, 0.004, 0.014), (-0.132, 0.1870, -0.029),
                           'bumper_left', BUMPER),
        'bumper_right': box((0.535, 0.004, 0.014), (-0.132, -0.1870, -0.029),
                            'bumper_right', BUMPER),
        'front_status_led': box((0.002, 0.018, 0.008), (0.1382, -0.100, -0.010),
                                'front_status_led', LED_RING),
        'number_left': decal(1.0, 'number_left'),
        'number_right': decal(-1.0, 'number_right'),
    }
    for name, mesh in parts.items():
        scene.add_geometry(mesh, geom_name=name, node_name=name)
    return scene


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path,
                        help='upstream ManiSkill-GoogleRobot checkout; omit to keep existing GLBs')
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
    if source is not None:
        if not source.is_dir():
            parser.error(f'--source is not a directory: {source}')
        for relative, options in visuals.items():
            emissive = source / options['emissive'] if options.get('emissive') else None
            faces = convert(source, relative, destination, gold=options['gold'],
                            tyre=options['tyre'], emissive=emissive)
            report[Path(relative).name + '.glb'] = faces
            print(f'{Path(relative).name}.glb  {faces} triangles')
    else:
        print('source omitted: keeping existing upstream GLBs')

    panel = head_panel()
    panel_path = destination / 'head_sensor_panel.glb'
    panel_path.write_bytes(panel.export(file_type='glb'))
    mark_unlit(panel_path)
    report['head_sensor_panel.glb'] = int(sum(len(g.faces) for g in panel.geometry.values()))
    print(f'head_sensor_panel.glb  {report["head_sensor_panel.glb"]} triangles')
    trim = base_trim()
    trim_path = destination / 'base_trim.glb'
    trim_path.write_bytes(trim.export(file_type='glb'))
    mark_unlit(trim_path, skip={'number_left', 'number_right'})
    report['base_trim.glb'] = int(sum(len(g.faces) for g in trim.geometry.values()))
    print(f'base_trim.glb  {report["base_trim.glb"]} triangles')
    args.appearance.parent.mkdir(parents=True, exist_ok=True)
    existing = {}
    if args.appearance.exists():
        try:
            existing = json.loads(args.appearance.read_text())
        except (OSError, json.JSONDecodeError):
            existing = {}
    triangles = dict(existing.get('triangles', {}))
    triangles.update(report)
    appearance = {
        **existing,
        'reference': 'Everyday Robots photo set (CNET kitchen tour)',
        'palette_srgb': {
            'shell': list(SHELL), 'finger_gold': list(GOLD), 'tyre': list(TYRE),
            'head_panel': {
                'plate': [12, 12, 14], 'bezel': [58, 58, 61],
                'lens_bezel': [74, 74, 77], 'lens_rim': [150, 150, 152],
                'lens_chrome': [206, 206, 209], 'lens_glass': [26, 30, 38],
                'lens_iris': [10, 12, 18], 'led': [248, 250, 255],
                'led_ring': list(LED_RING),
            },
            'base_trim': {'bumper': list(BUMPER), 'decal_ink': list(DECAL_INK)},
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
                           '(chrome ring on the centre lens), green LED ring and two status LEDs',
            'lens_centres_m': [-0.062, -0.031, 0.0, 0.031, 0.062],
        },
        'base_trim': {
            'file': 'meshes/habitat/base_trim.glb',
            'description': 'thin dark lower skirt, front green status light and 040 side '
                           'decals built as geometry (Habitat 0.3.3 ignores glTF alpha)',
        },
        'triangles': triangles,
    }
    args.appearance.write_text(json.dumps(appearance, indent=2) + '\n')


if __name__ == '__main__':
    main()
