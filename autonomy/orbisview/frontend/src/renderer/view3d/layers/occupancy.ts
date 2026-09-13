import * as THREE from 'three';
import type { OccupancyGridJson } from '../../map2d/types';
import { toThree } from '../coords';

const MAX_EDGE = 512;

function cellColor(v: number): [number, number, number, number] {
  if (v < 0) return [40, 48, 56, 90];
  if (v === 0) return [30, 40, 48, 40];
  if (v >= 100) return [200, 80, 80, 200];
  const t = v / 100;
  return [40 + 160 * t, 80, 120 - 60 * t, 80 + 100 * t];
}

export function updateOccupancyPlane(
  plane: THREE.Mesh,
  grid: OccupancyGridJson | null,
  opts: { opacity: number; yLift: number; visible: boolean },
): void {
  plane.visible = opts.visible && !!grid;
  if (!opts.visible || !grid) return;

  const { width, height, resolution, origin, data } = grid;
  if (width <= 0 || height <= 0 || resolution <= 0) {
    plane.visible = false;
    return;
  }

  const scale = Math.min(1, MAX_EDGE / Math.max(width, height));
  const tw = Math.max(1, Math.round(width * scale));
  const th = Math.max(1, Math.round(height * scale));
  const canvas = document.createElement('canvas');
  canvas.width = tw;
  canvas.height = th;
  const c2d = canvas.getContext('2d');
  if (!c2d) return;
  const img = c2d.createImageData(tw, th);
  for (let j = 0; j < th; j++) {
    for (let i = 0; i < tw; i++) {
      const sx = Math.min(width - 1, Math.floor(i / scale));
      const sy = Math.min(height - 1, Math.floor(j / scale));
      const v = data[sy * width + sx] ?? -1;
      const [r, g, b, a] = cellColor(v);
      const o = (j * tw + i) * 4;
      img.data[o] = r;
      img.data[o + 1] = g;
      img.data[o + 2] = b;
      img.data[o + 3] = a;
    }
  }
  c2d.putImageData(img, 0, 0);

  const prev = plane.material as THREE.MeshBasicMaterial;
  if (prev.map) prev.map.dispose();
  const tex = new THREE.CanvasTexture(canvas);
  tex.magFilter = THREE.NearestFilter;
  tex.minFilter = THREE.NearestFilter;
  tex.needsUpdate = true;
  prev.map = tex;
  prev.opacity = opts.opacity;
  prev.transparent = true;
  prev.needsUpdate = true;

  const worldW = width * resolution;
  const worldH = height * resolution;
  plane.geometry.dispose();
  plane.geometry = new THREE.PlaneGeometry(worldW, worldH);
  plane.rotation.x = -Math.PI / 2;
  const ox = origin?.x ?? 0;
  const oy = origin?.y ?? 0;
  const center = toThree(ox + worldW / 2, oy + worldH / 2, opts.yLift);
  plane.position.set(center.x, center.y, center.z);
}
