/** Planar yaw (rad, around +Z) → ROS-style quaternion (xyzw). */
export function yawToQuaternion(yaw: number): {
  x: number;
  y: number;
  z: number;
  w: number;
} {
  const half = yaw * 0.5;
  return { x: 0, y: 0, z: Math.sin(half), w: Math.cos(half) };
}

export function yawToDeg(yaw: number): number {
  return (yaw * 180) / Math.PI;
}

/** Clipboard / status text for Map pick tool. */
export function formatPickPose(x: number, y: number, yaw: number, z = 0): string {
  const q = yawToQuaternion(yaw);
  const deg = yawToDeg(yaw);
  return [
    `x=${x.toFixed(4)} y=${y.toFixed(4)} z=${z.toFixed(4)}`,
    `yaw=${yaw.toFixed(4)} rad (${deg.toFixed(1)}°)`,
    `qx=${q.x.toFixed(6)} qy=${q.y.toFixed(6)} qz=${q.z.toFixed(6)} qw=${q.w.toFixed(6)}`,
  ].join('\n');
}

export function pickPoseStatus(x: number, y: number, yaw: number): string {
  const q = yawToQuaternion(yaw);
  return (
    `取点 (${x.toFixed(2)}, ${y.toFixed(2)}) yaw ${yawToDeg(yaw).toFixed(0)}°` +
    ` · q(${q.x.toFixed(3)}, ${q.y.toFixed(3)}, ${q.z.toFixed(3)}, ${q.w.toFixed(3)}) · 已复制`
  );
}
