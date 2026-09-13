import type { ChassisJson, Pose2D } from './types';

function motionLabel(chassis: ChassisJson | null): string {
  const m = chassis?.motion_model;
  if (typeof m === 'number') return m === 1 ? 'ACKERMANN' : 'DIFF';
  const s = (m ?? 'DIFF').toString().toUpperCase();
  if (s.includes('ACKERMANN')) return 'ACKERMANN';
  return 'DIFF';
}

export function drawMapHud(
  ctx: CanvasRenderingContext2D,
  args: {
    pose: Pose2D | null;
    twist: { vx: number; wz: number } | null;
    chassis: ChassisJson | null;
    goal: Pose2D | null;
    footprintSource: 'stream' | 'default';
  },
): void {
  const model = motionLabel(args.chassis);
  const lines = [
    args.pose
      ? `pose ${args.pose.x.toFixed(2)} ${args.pose.y.toFixed(2)} ${(args.pose.yaw ?? 0).toFixed(2)}`
      : 'pose —',
    args.twist
      ? `model=${model} vx=${args.twist.vx.toFixed(2)} wz=${args.twist.wz.toFixed(2)}` +
        (model === 'ACKERMANN' && args.chassis?.steering != null
          ? ` steer=${args.chassis.steering.toFixed(2)}`
          : '')
      : `model=${model} twist —`,
    args.goal
      ? `goal ${args.goal.x.toFixed(2)} ${args.goal.y.toFixed(2)}`
      : 'goal —',
    `footprint=${args.footprintSource}`,
  ];
  const boxH = 14 + lines.length * 16;
  ctx.fillStyle = 'rgba(15,20,25,0.75)';
  ctx.fillRect(8, 8, 280, boxH);
  ctx.fillStyle = '#cfd8dc';
  ctx.font = '12px ui-monospace, monospace';
  lines.forEach((line, i) => {
    ctx.fillText(line, 14, 26 + i * 16);
  });
}
