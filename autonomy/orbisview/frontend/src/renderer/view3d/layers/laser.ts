import * as THREE from 'three';
import type { Pose2D } from '../../map2d/types';
import type { View3DContext } from '../createScene';
import { toThree } from '../coords';
import type { View3DLaserScan } from '../types';

export function updateLaser(
  ctx: View3DContext,
  pose: Pose2D | null,
  scan: View3DLaserScan | null,
  height: number,
  visible: boolean,
): void {
  ctx.laser.visible = visible && !!pose && !!scan?.ranges?.length;
  if (!visible || !pose || !scan?.ranges?.length) return;

  const yaw0 = pose.yaw ?? 0;
  const pts: number[] = [];
  for (let i = 0; i < scan.ranges.length; i++) {
    const r = scan.ranges[i];
    if (!Number.isFinite(r) || r <= 0) continue;
    const a = scan.angle_min + i * scan.angle_increment + yaw0;
    const wx = pose.x + r * Math.cos(a);
    const wy = pose.y + r * Math.sin(a);
    const t = toThree(wx, wy, height);
    pts.push(t.x, t.y, t.z);
  }
  const positions = new Float32Array(pts);
  ctx.laser.geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  ctx.laser.geometry.computeBoundingSphere();
}
