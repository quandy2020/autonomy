import type { SemanticZoneNorm } from './semanticZones';

export function drawSemanticZones(
  ctx: CanvasRenderingContext2D,
  zones: SemanticZoneNorm[],
  toScreen: (x: number, y: number) => readonly [number, number] | [number, number],
  scale: number,
): void {
  const list = zones;
  const showLabel = list.length <= 40 && scale >= 8;

  for (const z of list) {
    if (z.polygon.length < 3) continue;
    ctx.beginPath();
    z.polygon.forEach(([x, y], i) => {
      const [sx, sy] = toScreen(x, y);
      if (i === 0) ctx.moveTo(sx, sy);
      else ctx.lineTo(sx, sy);
    });
    ctx.closePath();
    ctx.fillStyle = z.fill;
    ctx.fill();
    ctx.strokeStyle = z.stroke;
    ctx.lineWidth = Math.max(1, z.strokeWidth);
    ctx.stroke();

    if (showLabel && z.label) {
      let cx = 0;
      let cy = 0;
      for (const [x, y] of z.polygon) {
        cx += x;
        cy += y;
      }
      cx /= z.polygon.length;
      cy /= z.polygon.length;
      const [sx, sy] = toScreen(cx, cy);
      ctx.fillStyle = 'rgba(236,239,241,0.92)';
      ctx.font = '11px system-ui, sans-serif';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(z.label, sx, sy);
    }
  }
}
