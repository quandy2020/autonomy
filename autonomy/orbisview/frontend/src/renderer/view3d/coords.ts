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
