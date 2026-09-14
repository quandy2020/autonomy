import { describe, expect, it } from 'vitest';
import {
  computeTextureSize,
  fillOccupancyRgba,
  occupancyCacheKey,
  occupancyCellRgba,
  occupancyToUint8,
} from './occupancyTexture';
import type { OccupancyGridJson } from './types';

describe('occupancyToUint8', () => {
  it('maps -1 to 255 like int8→uint8', () => {
    expect(occupancyToUint8(-1)).toBe(255);
  });

  it('keeps 0..100', () => {
    expect(occupancyToUint8(0)).toBe(0);
    expect(occupancyToUint8(100)).toBe(100);
  });
});

describe('occupancyCellRgba map (RViz makeMapPalette)', () => {
  it('unknown (-1) is RViz teal-gray 0x708986', () => {
    expect(occupancyCellRgba(-1, 'map')).toEqual([0x70, 0x89, 0x86, 255]);
  });

  it('free (0) is opaque white', () => {
    expect(occupancyCellRgba(0, 'map')).toEqual([255, 255, 255, 255]);
  });

  it('occupied (100) is opaque black', () => {
    expect(occupancyCellRgba(100, 'map')).toEqual([0, 0, 0, 255]);
  });

  it('mid value is gray scale', () => {
    const [r, g, b, a] = occupancyCellRgba(50, 'map');
    expect(r).toBe(g);
    expect(g).toBe(b);
    expect(r).toBe(255 - Math.floor((255 * 50) / 100));
    expect(a).toBe(255);
  });

  it('illegal 101 is green', () => {
    expect(occupancyCellRgba(101, 'map')).toEqual([0, 255, 0, 255]);
  });
});

describe('occupancyCellRgba costmap (RViz makeCostmapPalette)', () => {
  it('unknown (-1) is same teal-gray', () => {
    expect(occupancyCellRgba(-1, 'costmap')).toEqual([0x70, 0x89, 0x86, 255]);
  });

  it('free (0) is transparent', () => {
    expect(occupancyCellRgba(0, 'costmap')[3]).toBe(0);
  });

  it('lethal (100) is purple', () => {
    expect(occupancyCellRgba(100, 'costmap')).toEqual([255, 0, 255, 255]);
  });

  it('inscribed (99) is cyan', () => {
    expect(occupancyCellRgba(99, 'costmap')).toEqual([0, 255, 255, 255]);
  });

  it('cost 50 is blue→red blend', () => {
    const [r, g, b, a] = occupancyCellRgba(50, 'costmap');
    const v = Math.floor((255 * 50) / 100);
    expect(r).toBe(v);
    expect(g).toBe(0);
    expect(b).toBe(255 - v);
    expect(a).toBe(255);
  });
});

describe('computeTextureSize', () => {
  it('keeps small grids 1:1', () => {
    expect(computeTextureSize(10, 20)).toEqual({ tw: 10, th: 20, scale: 1 });
  });

  it('keeps large grids 1:1 by default (no downsampling)', () => {
    expect(computeTextureSize(2048, 1024)).toEqual({
      tw: 2048,
      th: 1024,
      scale: 1,
    });
  });

  it('honors an explicit finite maxEdge', () => {
    const { tw, th, scale } = computeTextureSize(2048, 1024, 1024);
    expect(Math.max(tw, th)).toBeLessThanOrEqual(1024);
    expect(scale).toBeLessThan(1);
  });
});

describe('fillOccupancyRgba', () => {
  it('writes RViz free/occupied pixels', () => {
    const grid: OccupancyGridJson = {
      resolution: 0.05,
      width: 2,
      height: 1,
      origin: { x: 0, y: 0 },
      data: [0, 100],
    };
    const out = new Uint8ClampedArray(2 * 1 * 4);
    fillOccupancyRgba(out, grid, 'map', 2, 1, 1);
    expect(Array.from(out.slice(0, 4))).toEqual([255, 255, 255, 255]);
    expect(Array.from(out.slice(4, 8))).toEqual([0, 0, 0, 255]);
  });
});

describe('occupancyCacheKey', () => {
  const base = (): OccupancyGridJson => ({
    resolution: 0.05,
    width: 2,
    height: 1,
    origin: { x: 1, y: 2 },
    data: [0, 100],
  });

  it('changes when mode changes', () => {
    const g = base();
    expect(occupancyCacheKey(g, 'map')).not.toBe(occupancyCacheKey(g, 'costmap'));
  });

  it('changes when a cell value changes', () => {
    const g = base();
    const a = occupancyCacheKey(g, 'map');
    g.data[1] = 50;
    expect(occupancyCacheKey(g, 'map')).not.toBe(a);
  });
});
