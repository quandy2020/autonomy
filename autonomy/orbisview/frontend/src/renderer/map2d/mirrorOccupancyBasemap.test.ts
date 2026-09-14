import { describe, expect, it } from 'vitest';
import { occupancyCellRgba } from './occupancyTexture';
import { occupancyGridRgba } from './mirrorOccupancyBasemap';
import type { OccupancyGridJson } from './types';

describe('occupancyGridRgba', () => {
  it('matches grid size for small maps and paints occupied cell', () => {
    const grid: OccupancyGridJson = {
      resolution: 0.05,
      width: 2,
      height: 1,
      origin: { x: -1, y: -1 },
      data: [0, 100],
    };
    const { tw, th, data } = occupancyGridRgba(grid);
    expect(tw).toBe(2);
    expect(th).toBe(1);
    expect(Array.from(data.slice(0, 4))).toEqual([255, 255, 255, 255]); // free = RViz white
    const expected = occupancyCellRgba(100, 'map');
    expect(Array.from(data.slice(4, 8))).toEqual([...expected]); // occupied = black
  });
});
