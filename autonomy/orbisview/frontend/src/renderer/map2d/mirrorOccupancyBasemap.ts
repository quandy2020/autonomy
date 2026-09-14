import {
  computeTextureSize,
  fillOccupancyRgba,
  OCCUPANCY_MAX_EDGE,
} from './occupancyTexture';
import type { OccupancyGridJson } from './types';
import type { StaticSlamBasemap } from './staticSlam';

export function occupancyGridRgba(
  grid: OccupancyGridJson,
  maxEdge = OCCUPANCY_MAX_EDGE,
): { tw: number; th: number; data: Uint8ClampedArray } {
  const { tw, th, scale } = computeTextureSize(grid.width, grid.height, maxEdge);
  const data = new Uint8ClampedArray(tw * th * 4);
  fillOccupancyRgba(data, grid, 'map', tw, th, scale);
  return { tw, th, data };
}

/** Paint map-mode occupancy into an offscreen canvas (RGBA). */
export function occupancyGridToCanvas(
  grid: OccupancyGridJson,
  maxEdge = OCCUPANCY_MAX_EDGE,
): HTMLCanvasElement {
  const { tw, th, data } = occupancyGridRgba(grid, maxEdge);
  const canvas = document.createElement('canvas');
  canvas.width = tw;
  canvas.height = th;
  const ctx = canvas.getContext('2d');
  if (!ctx) throw new Error('2d context unavailable');
  const img = ctx.createImageData(tw, th);
  img.data.set(data);
  ctx.putImageData(img, 0, 0);
  return canvas;
}

/**
 * Build StaticSlamBasemap whose imageSrc is a blob: URL of the canvas PNG.
 * Caller owns revoke of previous mirror blob via setBasemap(revokePrevious).
 */
export async function basemapFromOccupancyGrid(
  grid: OccupancyGridJson,
  label = 'live-mirror',
): Promise<StaticSlamBasemap> {
  if (grid.width <= 0 || grid.height <= 0 || !(grid.resolution > 0)) {
    throw new Error('invalid occupancy grid');
  }
  const canvas = occupancyGridToCanvas(grid);
  const blob = await new Promise<Blob>((resolve, reject) => {
    canvas.toBlob(
      (b) => (b ? resolve(b) : reject(new Error('canvas.toBlob failed'))),
      'image/png',
    );
  });
  const imageSrc = URL.createObjectURL(blob);
  return {
    imageSrc,
    originX: grid.origin?.x ?? 0,
    originY: grid.origin?.y ?? 0,
    resolution: grid.resolution,
    widthPx: grid.width,
    heightPx: grid.height,
    label,
    source: 'file',
  };
}
