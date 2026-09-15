export function toThree(
  x: number,
  y: number,
  z = 0,
): { x: number; y: number; z: number } {
  return { x, y: z, z: -y };
}

/** Inverse of {@link toThree} (map-frame x/y from Three position). */
export function fromThree(tx: number, _ty: number, tz: number): { x: number; y: number } {
  return { x: tx, y: -tz };
}

/** Map-frame yaw (ROS: 0 = +X, CCW) → unit forward in Three.js after {@link toThree}. */
export function yawToThreeForward(yaw: number): { x: number; y: number; z: number } {
  return { x: Math.cos(yaw), y: 0, z: -Math.sin(yaw) };
}
