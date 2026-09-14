import * as THREE from 'three';
import type { View3DContext } from '../createScene';
import { toThree } from '../coords';
import type { CloudColorMode, View3DCloudPoint } from '../types';

export function updateCloud(
  ctx: View3DContext,
  points: View3DCloudPoint[] | null,
  visible: boolean,
  mode: CloudColorMode,
  opts?: { size?: number },
): void {
  ctx.cloud.visible = visible && !!points?.length;
  if (!visible || !points?.length) return;

  if (opts?.size != null) {
    (ctx.cloud.material as THREE.PointsMaterial).size = opts.size;
  }

  const n = points.length;
  const positions = new Float32Array(n * 3);
  const colors = new Float32Array(n * 3);
  let zMin = Infinity;
  let zMax = -Infinity;
  if (mode === 'height') {
    for (const p of points) {
      zMin = Math.min(zMin, p.z);
      zMax = Math.max(zMax, p.z);
    }
  }
  const zSpan = Math.max(1e-6, zMax - zMin);

  for (let i = 0; i < n; i++) {
    const p = points[i];
    const t = toThree(p.x, p.y, p.z);
    positions[i * 3] = t.x;
    positions[i * 3 + 1] = t.y;
    positions[i * 3 + 2] = t.z;
    if (mode === 'intensity') {
      const inten = p.i ?? 0.7;
      colors[i * 3] = 0.2 + 0.8 * inten;
      colors[i * 3 + 1] = 0.6 * inten;
      colors[i * 3 + 2] = 1 - 0.5 * inten;
    } else {
      const t01 = (p.z - zMin) / zSpan;
      colors[i * 3] = t01;
      colors[i * 3 + 1] = 0.4 + 0.4 * (1 - t01);
      colors[i * 3 + 2] = 1 - t01;
    }
  }

  ctx.cloud.geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  ctx.cloud.geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
  ctx.cloud.geometry.computeBoundingSphere();
}
