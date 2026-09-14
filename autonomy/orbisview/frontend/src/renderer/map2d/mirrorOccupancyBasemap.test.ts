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
    expect(data[3]).toBe(0); // free
    const expected = occupancyCellRgba(100, 'map');
    expect(data[4]).toBe(expected[0]);
    expect(data[5]).toBe(expected[1]);
    expect(data[6]).toBe(expected[2]);
    expect(data[7]).toBe(expected[3]);
  });
});
