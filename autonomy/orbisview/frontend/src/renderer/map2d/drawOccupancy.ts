import type { OccupancyGridJson } from './types';
import type { WorldToScreen } from './coords';
import {
  OccupancyTextureCache,
  type OccupancyPaintMode,
} from './occupancyTexture';

const defaultMapCache = new OccupancyTextureCache();
const defaultCostmapCache = new OccupancyTextureCache();

export function drawOccupancyGrid(
  ctx: CanvasRenderingContext2D,
  grid: OccupancyGridJson,
  toScreen: WorldToScreen,
  _scale: number,
  mode: OccupancyPaintMode,
  cache?: OccupancyTextureCache,
): void {
  const expected = grid.width * grid.height;
  const data = grid.data ?? [];
  if (data.length !== expected && !(globalThis as { __ovGridWarn?: boolean }).__ovGridWarn) {
    console.warn(
      `[orbisview] OccupancyGrid data length ${data.length} != ${expected}; truncating/padding`,
    );
    (globalThis as { __ovGridWarn?: boolean }).__ovGridWarn = true;
  }

  const c = cache ?? (mode === 'costmap' ? defaultCostmapCache : defaultMapCache);
  const tex = c.get(grid, mode);
  if (!tex) return;

  const x0 = tex.originX;
  const y0 = tex.originY;
  const x1 = x0 + tex.worldW;
  const y1 = y0 + tex.worldH;
  const [sx0, sy0] = toScreen(x0, y0);
  const [sx1, sy1] = toScreen(x1, y1);
  const left = Math.min(sx0, sx1);
  const top = Math.min(sy0, sy1);
  const w = Math.abs(sx1 - sx0);
  const h = Math.abs(sy1 - sy0);
  ctx.imageSmoothingEnabled = false;
  // Texture row 0 = grid y=0 (world bottom); screen Y grows downward → flip.
  ctx.save();
  ctx.translate(left, top + h);
  ctx.scale(1, -1);
  ctx.drawImage(tex.canvas, 0, 0, w, h);
  ctx.restore();
}
