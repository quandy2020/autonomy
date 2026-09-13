import type { OccupancyGridJson } from './types';
import type { WorldToScreen } from './coords';

export function drawOccupancyGrid(
  ctx: CanvasRenderingContext2D,
  grid: OccupancyGridJson,
  toScreen: WorldToScreen,
  scale: number,
  mode: 'map' | 'costmap',
): void {
  const expected = grid.width * grid.height;
  const data = grid.data ?? [];
  if (data.length !== expected && !(globalThis as { __ovGridWarn?: boolean }).__ovGridWarn) {
    console.warn(
      `[orbisview] OccupancyGrid data length ${data.length} != ${expected}; truncating/padding`,
    );
    (globalThis as { __ovGridWarn?: boolean }).__ovGridWarn = true;
  }

  for (let gy = 0; gy < grid.height; gy++) {
    for (let gx = 0; gx < grid.width; gx++) {
      const v = data[gy * grid.width + gx] ?? 0;
      if (v === 0) continue;
      const wx = grid.origin.x + gx * grid.resolution;
      const wy = grid.origin.y + gy * grid.resolution;
      const [sx, sy] = toScreen(wx, wy);
      const cell = grid.resolution * scale;
      if (mode === 'costmap') {
        if (v < 0) continue;
        ctx.fillStyle = v >= 100 ? 'rgba(255,112,67,0.55)' : 'rgba(255,167,38,0.35)';
      } else {
        ctx.fillStyle = v < 0 ? '#334' : '#a33';
      }
      ctx.fillRect(sx, sy - cell, cell, cell);
    }
  }
}
