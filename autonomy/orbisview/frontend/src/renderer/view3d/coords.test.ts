import { describe, expect, it } from 'vitest';
import { toThree } from './coords';

describe('toThree', () => {
  it('maps map XY to Three X/−Z with optional height on Y', () => {
    expect(toThree(1, 2)).toEqual({ x: 1, y: 0, z: -2 });
    expect(toThree(1, 2, 0.5)).toEqual({ x: 1, y: 0.5, z: -2 });
  });
});
