import { describe, expect, it } from 'vitest';
import {
  computeTextureSize,
  fillOccupancyRgba,
  occupancyCellRgba,
  OCCUPANCY_MAX_EDGE,
} from './occupancyTexture';
import type { OccupancyGridJson } from './types';

describe('occupancyCellRgba map', () => {
  it('unknown is translucent gray', () => {
    const [r, g, , a] = occupancyCellRgba(-1, 'map');
    expect(a).toBeGreaterThan(0);
    expect(a).toBeLessThan(255);
    expect(Math.abs(r - g)).toBeLessThan(20);
  });

  it('free is fully transparent', () => {
    expect(occupancyCellRgba(0, 'map')[3]).toBe(0);
  });

  it('occupied is dark opaque-ish', () => {
    const [r, g, b, a] = occupancyCellRgba(100, 'map');
    expect(a).toBeGreaterThan(180);
    expect(r).toBeLessThan(80);
    expect(g).toBeLessThan(80);
    expect(b).toBeLessThan(80);
  });
});

describe('occupancyCellRgba costmap', () => {
  it('skips unknown (alpha 0)', () => {
    expect(occupancyCellRgba(-1, 'costmap')[3]).toBe(0);
  });

  it('lethal is warm', () => {
    const [r, g, , a] = occupancyCellRgba(100, 'costmap');
    expect(a).toBeGreaterThan(80);
    expect(r).toBeGreaterThan(g);
  });
});

describe('computeTextureSize', () => {
  it('keeps small grids 1:1', () => {
    expect(computeTextureSize(10, 20)).toEqual({ tw: 10, th: 20, scale: 1 });
  });

  it('downsamples when over max edge', () => {
    const { tw, th, scale } = computeTextureSize(2048, 1024, OCCUPANCY_MAX_EDGE);
    expect(Math.max(tw, th)).toBeLessThanOrEqual(OCCUPANCY_MAX_EDGE);
    expect(scale).toBeLessThan(1);
  });
});

describe('fillOccupancyRgba', () => {
  it('writes free cell as transparent', () => {
    const grid: OccupancyGridJson = {
      resolution: 0.05,
      width: 2,
      height: 1,
      origin: { x: 0, y: 0 },
      data: [0, 100],
    };
    const out = new Uint8ClampedArray(2 * 1 * 4);
    fillOccupancyRgba(out, grid, 'map', 2, 1, 1);
    expect(out[3]).toBe(0);
    expect(out[7]).toBeGreaterThan(180);
  });
});
