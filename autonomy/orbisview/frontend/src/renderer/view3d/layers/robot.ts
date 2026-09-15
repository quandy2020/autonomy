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
  const p = toThree(pose.x, pose.y, 0.05);
  ctx.robot.position.set(p.x, p.y, p.z);
  // Group +X arrow; R_y(yaw) → (cos yaw, 0, -sin yaw) in Three = map forward.
  ctx.robot.rotation.set(0, pose.yaw ?? 0, 0);
}
