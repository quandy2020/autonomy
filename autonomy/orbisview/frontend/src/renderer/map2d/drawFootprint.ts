import type { DefaultFootprint, Pose2D, RobotFootprintJson } from './types';
import type { WorldToScreen } from './coords';

function shapeName(shape: string | number | undefined): string {
  if (typeof shape === 'number') {
    if (shape === 2) return 'CIRCLE';
    if (shape === 1) return 'POLYGON';
    return 'RECT';
  }
  const s = (shape ?? 'RECT').toString().toUpperCase();
  if (s.includes('CIRCLE')) return 'CIRCLE';
  if (s.includes('POLYGON')) return 'POLYGON';
  return 'RECT';
}

export function resolveFootprintPoints(
  fp: RobotFootprintJson | null,
  fallback: DefaultFootprint,
): { x: number; y: number }[] {
  if (fp) {
    const kind = shapeName(fp.shape);
    if (kind === 'CIRCLE') {
      const r = fp.radius ?? Math.max(fallback.length, fallback.width) / 2;
      const pts: { x: number; y: number }[] = [];
      for (let i = 0; i < 16; i++) {
        const a = (i / 16) * Math.PI * 2;
        pts.push({ x: Math.cos(a) * r, y: Math.sin(a) * r });
      }
      return pts;
    }
    if (fp.points && fp.points.length >= 3) {
      return fp.points.map((p) => ({ x: p.x, y: p.y }));
    }
    if (kind === 'RECT' || fp.length || fp.width) {
      const L = (fp.length ?? fallback.length) / 2;
      const W = (fp.width ?? fallback.width) / 2;
      return [
        { x: L, y: W },
        { x: L, y: -W },
        { x: -L, y: -W },
        { x: -L, y: W },
      ];
    }
  }
  const L = fallback.length / 2;
  const W = fallback.width / 2;
  return [
    { x: L, y: W },
    { x: L, y: -W },
    { x: -L, y: -W },
    { x: -L, y: W },
  ];
}

export function drawFootprint(
  ctx: CanvasRenderingContext2D,
  pose: Pose2D,
  pointsBody: { x: number; y: number }[],
  toScreen: WorldToScreen,
): void {
  const yaw = pose.yaw ?? 0;
  const c = Math.cos(yaw);
  const s = Math.sin(yaw);
  ctx.strokeStyle = '#80cbc4';
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  pointsBody.forEach((p, i) => {
    const wx = pose.x + p.x * c - p.y * s;
    const wy = pose.y + p.x * s + p.y * c;
    const [sx, sy] = toScreen(wx, wy);
    if (i === 0) ctx.moveTo(sx, sy);
    else ctx.lineTo(sx, sy);
  });
  ctx.closePath();
  ctx.stroke();
}
