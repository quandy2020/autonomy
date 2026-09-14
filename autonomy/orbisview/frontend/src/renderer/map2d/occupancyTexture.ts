import type { OccupancyGridJson } from './types';

/**
 * Color schemes matching ROS 2 RViz Map display
 * (`rviz_default_plugins/.../map/palette_builder.cpp`).
 */
export type OccupancyPaintMode = 'map' | 'costmap' | 'raw';
/** No spatial downsampling — texture size equals grid width/height. */
export const OCCUPANCY_MAX_EDGE = Number.POSITIVE_INFINITY;

/** RViz unknown (-1 as uint8 255): teal-gray. */
const RVIZ_UNKNOWN: [number, number, number, number] = [0x70, 0x89, 0x86, 255];

function clampInt8(v: number): number {
  let i = Math.trunc(Number(v));
  if (!Number.isFinite(i)) i = -1;
  if (i > 127) i = 127;
  if (i < -128) i = -128;
  return i;
}

/** int8 cell → uint8 palette index (same as RViz MapDisplay). */
export function occupancyToUint8(v: number): number {
  return clampInt8(v) & 0xff;
}

function buildMapPalette(): Uint8ClampedArray {
  const p = new Uint8ClampedArray(256 * 4);
  for (let i = 0; i <= 100; i++) {
    const g = 255 - Math.floor((255 * i) / 100);
    const o = i * 4;
    p[o] = g;
    p[o + 1] = g;
    p[o + 2] = g;
    p[o + 3] = 255;
  }
  for (let i = 101; i <= 127; i++) {
    const o = i * 4;
    p[o] = 0;
    p[o + 1] = 255;
    p[o + 2] = 0;
    p[o + 3] = 255;
  }
  for (let i = 128; i <= 254; i++) {
    const o = i * 4;
    p[o] = 255;
    p[o + 1] = Math.floor((255 * (i - 128)) / (254 - 128));
    p[o + 2] = 0;
    p[o + 3] = 255;
  }
  const u = 255 * 4;
  p[u] = RVIZ_UNKNOWN[0];
  p[u + 1] = RVIZ_UNKNOWN[1];
  p[u + 2] = RVIZ_UNKNOWN[2];
  p[u + 3] = RVIZ_UNKNOWN[3];
  return p;
}

function buildCostmapPalette(): Uint8ClampedArray {
  const p = new Uint8ClampedArray(256 * 4);
  // free → transparent
  p[0] = 0;
  p[1] = 0;
  p[2] = 0;
  p[3] = 0;
  for (let i = 1; i <= 98; i++) {
    const v = Math.floor((255 * i) / 100);
    const o = i * 4;
    p[o] = v;
    p[o + 1] = 0;
    p[o + 2] = 255 - v;
    p[o + 3] = 255;
  }
  // 99 inscribed / inflated obstacle → cyan
  p[99 * 4] = 0;
  p[99 * 4 + 1] = 255;
  p[99 * 4 + 2] = 255;
  p[99 * 4 + 3] = 255;
  // 100 lethal → purple
  p[100 * 4] = 255;
  p[100 * 4 + 1] = 0;
  p[100 * 4 + 2] = 255;
  p[100 * 4 + 3] = 255;
  for (let i = 101; i <= 127; i++) {
    const o = i * 4;
    p[o] = 0;
    p[o + 1] = 255;
    p[o + 2] = 0;
    p[o + 3] = 255;
  }
  for (let i = 128; i <= 254; i++) {
    const o = i * 4;
    p[o] = 255;
    p[o + 1] = Math.floor((255 * (i - 128)) / (254 - 128));
    p[o + 2] = 0;
    p[o + 3] = 255;
  }
  const u = 255 * 4;
  p[u] = RVIZ_UNKNOWN[0];
  p[u + 1] = RVIZ_UNKNOWN[1];
  p[u + 2] = RVIZ_UNKNOWN[2];
  p[u + 3] = RVIZ_UNKNOWN[3];
  return p;
}

function buildRawPalette(): Uint8ClampedArray {
  const p = new Uint8ClampedArray(256 * 4);
  for (let i = 0; i < 256; i++) {
    const o = i * 4;
    p[o] = i;
    p[o + 1] = i;
    p[o + 2] = i;
    p[o + 3] = 255;
  }
  return p;
}

const PALETTES: Record<OccupancyPaintMode, Uint8ClampedArray> = {
  map: buildMapPalette(),
  costmap: buildCostmapPalette(),
  raw: buildRawPalette(),
};

export function occupancyCellRgba(
  v: number,
  mode: OccupancyPaintMode,
): [number, number, number, number] {
  const idx = occupancyToUint8(v);
  const p = PALETTES[mode] ?? PALETTES.map;
  const o = idx * 4;
  return [p[o], p[o + 1], p[o + 2], p[o + 3]];
}

export function computeTextureSize(
  width: number,
  height: number,
  maxEdge = OCCUPANCY_MAX_EDGE,
): { tw: number; th: number; scale: number } {
  if (width <= 0 || height <= 0) return { tw: 1, th: 1, scale: 1 };
  if (!Number.isFinite(maxEdge) || maxEdge <= 0) {
    return { tw: width, th: height, scale: 1 };
  }
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
  const palette = PALETTES[mode] ?? PALETTES.map;
  for (let j = 0; j < th; j++) {
    for (let i = 0; i < tw; i++) {
      const sx = Math.min(width - 1, Math.floor(i / scale));
      const sy = Math.min(height - 1, Math.floor(j / scale));
      const v = data[sy * width + sx] ?? -1;
      const idx = occupancyToUint8(v);
      const src = idx * 4;
      const o = (j * tw + i) * 4;
      out[o] = palette[src];
      out[o + 1] = palette[src + 1];
      out[o + 2] = palette[src + 2];
      out[o + 3] = palette[src + 3];
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
