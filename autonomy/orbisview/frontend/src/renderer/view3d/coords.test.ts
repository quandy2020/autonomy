import { describe, expect, it } from 'vitest';
import { fromThree, toThree } from './coords';

describe('toThree', () => {
  it('maps map XY to Three X/−Z with optional height on Y', () => {
    expect(toThree(1, 2)).toEqual({ x: 1, y: 0, z: -2 });
    expect(toThree(1, 2, 0.5)).toEqual({ x: 1, y: 0.5, z: -2 });
  });
});

describe('fromThree', () => {
  it('inverts toThree for map XY', () => {
    const t = toThree(3.5, -1.2, 0.4);
    expect(fromThree(t.x, t.y, t.z)).toEqual({ x: 3.5, y: -1.2 });
  });
});
