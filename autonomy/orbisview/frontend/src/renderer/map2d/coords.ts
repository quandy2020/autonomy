export type WorldToScreen = (x: number, y: number) => readonly [number, number];

export function makeWorldToScreen(
  canvasW: number,
  canvasH: number,
  offsetX: number,
  offsetY: number,
  scale: number,
): WorldToScreen {
  const cx = canvasW / 2;
  const cy = canvasH / 2;
  return (x, y) => [cx + (x - offsetX) * scale, cy - (y - offsetY) * scale] as const;
}
