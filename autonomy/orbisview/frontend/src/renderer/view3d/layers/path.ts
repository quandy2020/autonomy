import * as THREE from 'three';
import { LineGeometry } from 'three/examples/jsm/lines/LineGeometry.js';
import type { View3DContext } from '../createScene';
import { toThree } from '../coords';
import type { Pose2D } from '../../map2d/types';
import type { View3DNavGoal } from '../types';

export function updatePathAndGoal(
  ctx: View3DContext,
  pose: Pose2D | null,
  path: Pose2D[] | null,
  goal: View3DNavGoal | null,
  pathVisible: boolean,
  style?: { color?: number; opacity?: number; lineWidth?: number },
): void {
  const showPath = pathVisible && !!path && path.length >= 2;
  ctx.path.visible = showPath;
  if (showPath && path) {
    const mat = ctx.pathMaterial;
    if (style?.color != null) mat.color.setHex(style.color);
    if (style?.opacity != null) {
      mat.transparent = style.opacity < 1;
      mat.opacity = Math.max(0, Math.min(1, style.opacity));
    }
    if (style?.lineWidth != null) {
      mat.linewidth = Math.max(1, style.lineWidth);
    }

    const positions: number[] = [];
    for (const p of path) {
      const t = toThree(p.x, p.y, 0.05);
      positions.push(t.x, t.y, t.z);
    }
    const geom = ctx.path.geometry as LineGeometry;
    geom.setPositions(positions);
    // Solid Line2 does not need distances; only call after real segments exist.
    if (positions.length >= 6) ctx.path.computeLineDistances();
  }

  const showGoal = !!goal;
  ctx.goal.visible = showGoal;
  ctx.goalLine.visible = showGoal && !!pose;
  if (showGoal && goal) {
    const g = toThree(goal.x, goal.y, 0.3);
    ctx.goal.position.set(g.x, g.y, g.z);
    if (pose) {
      const a = toThree(pose.x, pose.y, 0.15);
      const b = toThree(goal.x, goal.y, 0.15);
      ctx.goalLine.geometry.setFromPoints([
        new THREE.Vector3(a.x, a.y, a.z),
        new THREE.Vector3(b.x, b.y, b.z),
      ]);
      ctx.goalLine.computeLineDistances();
    }
  }
}
