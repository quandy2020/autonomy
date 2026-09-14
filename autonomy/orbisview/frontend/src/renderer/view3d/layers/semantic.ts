import * as THREE from 'three';
import type { SemanticZoneNorm } from '../../map2d/semanticZones';
import { toThree } from '../coords';

function parseRgba(css: string): { r: number; g: number; b: number; a: number } {
  const m = css.match(
    /rgba?\(\s*([\d.]+)\s*,\s*([\d.]+)\s*,\s*([\d.]+)(?:\s*,\s*([\d.]+))?\s*\)/i,
  );
  if (!m) return { r: 0.5, g: 0.5, b: 0.5, a: 0.3 };
  return {
    r: Number(m[1]) / 255,
    g: Number(m[2]) / 255,
    b: Number(m[3]) / 255,
    a: m[4] != null ? Number(m[4]) : 1,
  };
}

function zoneKey(zones: SemanticZoneNorm[]): string {
  return zones
    .map((z) => `${z.id}:${z.polygon.length}:${z.fill}`)
    .join('|');
}

let lastKey = '';

export function updateSemanticZones(
  group: THREE.Group,
  zones: SemanticZoneNorm[] | null | undefined,
  visible: boolean,
): void {
  group.visible = visible && !!zones?.length;
  if (!visible || !zones?.length) {
    clearGroup(group);
    lastKey = '';
    return;
  }

  const key = zoneKey(zones);
  if (key === lastKey) return;
  lastKey = key;
  clearGroup(group);

  for (const z of zones) {
    if (z.polygon.length < 3) continue;
    // Shape XY → after rotation.x=-π/2 matches toThree(mapX, mapY)
    const shape = new THREE.Shape();
    z.polygon.forEach(([x, y], i) => {
      if (i === 0) shape.moveTo(x, y);
      else shape.lineTo(x, y);
    });
    const geom = new THREE.ShapeGeometry(shape);
    const fill = parseRgba(z.fill);
    const mat = new THREE.MeshBasicMaterial({
      color: new THREE.Color(fill.r, fill.g, fill.b),
      transparent: true,
      opacity: Math.max(0.05, Math.min(1, fill.a)),
      depthWrite: false,
      side: THREE.DoubleSide,
    });
    const mesh = new THREE.Mesh(geom, mat);
    mesh.rotation.x = -Math.PI / 2;
    mesh.position.y = 0.015;
    group.add(mesh);

    const pts = z.polygon.map(([x, y]) => {
      const p = toThree(x, y, 0.016);
      return new THREE.Vector3(p.x, p.y, p.z);
    });
    pts.push(pts[0].clone());
    const lineGeom = new THREE.BufferGeometry().setFromPoints(pts);
    const stroke = parseRgba(z.stroke);
    group.add(
      new THREE.Line(
        lineGeom,
        new THREE.LineBasicMaterial({
          color: new THREE.Color(stroke.r, stroke.g, stroke.b),
          transparent: true,
          opacity: Math.max(0.2, Math.min(1, stroke.a)),
        }),
      ),
    );
  }
}

function clearGroup(group: THREE.Group) {
  while (group.children.length) {
    const c = group.children[0];
    group.remove(c);
    c.traverse((o) => {
      const mesh = o as THREE.Mesh | THREE.Line;
      if (mesh.geometry) mesh.geometry.dispose();
      const mat = (mesh as THREE.Mesh).material;
      if (!mat) return;
      if (Array.isArray(mat)) mat.forEach((m) => m.dispose());
      else (mat as THREE.Material).dispose();
    });
  }
}
