import * as THREE from 'three';
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
): void {
  ctx.path.visible = pathVisible && !!path?.length;
  if (pathVisible && path?.length) {
    const pts = path.map((p) => {
      const t = toThree(p.x, p.y, 0.05);
      return new THREE.Vector3(t.x, t.y, t.z);
    });
    ctx.path.geometry.setFromPoints(pts);
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
