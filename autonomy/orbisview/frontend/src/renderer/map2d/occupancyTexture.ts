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
