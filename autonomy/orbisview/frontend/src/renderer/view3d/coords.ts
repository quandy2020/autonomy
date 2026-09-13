export function toThree(
  x: number,
  y: number,
  z = 0,
): { x: number; y: number; z: number } {
  return { x, y: z, z: -y };
}
