import type { OccupancyGridJson } from './types';

export type OccupancyPaintMode = 'map' | 'costmap';
export const OCCUPANCY_MAX_EDGE = 1024;

export function occupancyCellRgba(
  v: number,
  mode: OccupancyPaintMode,
): [number, number, number, number] {
  if (mode === 'costmap') {
    if (v < 0) return [0, 0, 0, 0];
    if (v >= 100) return [255, 112, 67, 140];
    if (v === 0) return [0, 0, 0, 0];
    return [255, 167, 38, 90];
  }
  if (v < 0) return [160, 168, 176, 110];
  if (v === 0) return [0, 0, 0, 0];
  if (v >= 100) return [40, 44, 52, 230];
  const t = v / 100;
  return [
    Math.round(40 * t),
    Math.round(44 * t),
    Math.round(52 * t),
    Math.round(40 + 190 * t),
  ];
}

export function computeTextureSize(
  width: number,
  height: number,
  maxEdge = OCCUPANCY_MAX_EDGE,
): { tw: number; th: number; scale: number } {
  if (width <= 0 || height <= 0) return { tw: 1, th: 1, scale: 1 };
  const scale = Math.min(1, maxEdge / Math.max(width, height));
  return {
    tw: Math.max(1, Math.round(width * scale)),
    th: Math.max(1, Math.round(height * scale)),
    scale,
  };
}

export function fillOccupancyRgba(
  out: Uint8ClampedArray,
  grid: OccupancyGridJson,
  mode: OccupancyPaintMode,
  tw: number,
  th: number,
  scale: number,
): void {
  const { width, height, data } = grid;
  for (let j = 0; j < th; j++) {
    for (let i = 0; i < tw; i++) {
      const sx = Math.min(width - 1, Math.floor(i / scale));
      const sy = Math.min(height - 1, Math.floor(j / scale));
      const v = data[sy * width + sx] ?? -1;
      const [r, g, b, a] = occupancyCellRgba(v, mode);
      const o = (j * tw + i) * 4;
      out[o] = r;
      out[o + 1] = g;
      out[o + 2] = b;
      out[o + 3] = a;
    }
  }
}

export function occupancyCacheKey(
  grid: OccupancyGridJson,
  mode: OccupancyPaintMode,
): string {
  const data = grid.data ?? [];
  const mid = Math.floor(data.length / 2);
  return [
    mode,
    grid.width,
    grid.height,
    grid.resolution,
    grid.origin?.x ?? 0,
    grid.origin?.y ?? 0,
    data.length,
    data[0] ?? '',
    data[mid] ?? '',
    data[data.length - 1] ?? '',
  ].join('|');
}

export interface OccupancyTextureHandle {
  canvas: HTMLCanvasElement;
  texW: number;
  texH: number;
  worldW: number;
  worldH: number;
  originX: number;
  originY: number;
}

export class OccupancyTextureCache {
  private key = '';
  private handle: OccupancyTextureHandle | null = null;

  get(
    grid: OccupancyGridJson | null | undefined,
    mode: OccupancyPaintMode,
  ): OccupancyTextureHandle | null {
    if (!grid || grid.width <= 0 || grid.height <= 0 || !(grid.resolution > 0)) {
      return null;
    }
    const next = occupancyCacheKey(grid, mode);
    if (this.handle && this.key === next) return this.handle;

    const { tw, th, scale } = computeTextureSize(grid.width, grid.height);
    const canvas = document.createElement('canvas');
    canvas.width = tw;
    canvas.height = th;
    const ctx = canvas.getContext('2d');
    if (!ctx) return null;
    const img = ctx.createImageData(tw, th);
    fillOccupancyRgba(img.data, grid, mode, tw, th, scale);
    ctx.putImageData(img, 0, 0);

    const originX = grid.origin?.x ?? 0;
    const originY = grid.origin?.y ?? 0;
    this.handle = {
      canvas,
      texW: tw,
      texH: th,
      worldW: grid.width * grid.resolution,
      worldH: grid.height * grid.resolution,
      originX,
      originY,
    };
    this.key = next;
    return this.handle;
  }

  clear(): void {
    this.key = '';
    this.handle = null;
  }
}
