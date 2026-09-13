import * as THREE from 'three';
import { DEFAULT_FOOTPRINT } from '@/config/parameters';
import { resolveFootprintPoints } from '@/renderer/map2d/drawFootprint';
import type { Pose2D, RobotFootprintJson } from '../../map2d/types';
import type { View3DContext } from '../createScene';
import { toThree } from '../coords';

export function updateFootprint(
  ctx: View3DContext,
  pose: Pose2D | null,
  footprint: RobotFootprintJson | null,
  visible: boolean,
): void {
  ctx.footprint.visible = visible && !!pose;
  if (!visible || !pose) return;

  const body = resolveFootprintPoints(footprint, DEFAULT_FOOTPRINT);
  const yaw = pose.yaw ?? 0;
  const c = Math.cos(yaw);
  const s = Math.sin(yaw);
  const pts = body.map((p) => {
    const wx = pose.x + p.x * c - p.y * s;
    const wy = pose.y + p.x * s + p.y * c;
    const t = toThree(wx, wy, 0.03);
    return new THREE.Vector3(t.x, t.y, t.z);
  });
  ctx.footprint.geometry.setFromPoints(pts);
}
