import type { View3DContext } from '../createScene';
import { toThree } from '../coords';
import type { Pose2D } from '../../map2d/types';

export function updateRobot(
  ctx: View3DContext,
  pose: Pose2D | null,
  visible: boolean,
): void {
  ctx.robot.visible = visible && !!pose;
  if (!pose || !visible) return;
  const p = toThree(pose.x, pose.y, 0.2);
  ctx.robot.position.set(p.x, p.y, p.z);
  ctx.robot.rotation.z = -(pose.yaw ?? 0);
}
