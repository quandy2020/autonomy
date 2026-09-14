import { describe, expect, it } from 'vitest';
import { decodePgm } from './pgmDecode';

describe('decodePgm', () => {
  it('decodes ASCII P2 2x1', () => {
    const text = 'P2\n2 1\n255\n0 255\n';
    const img = decodePgm(new TextEncoder().encode(text).buffer);
    expect(img.width).toBe(2);
    expect(img.height).toBe(1);
    expect(img.data[0]).toBe(0);
    expect(img.data[4]).toBe(255);
    expect(img.data[7]).toBe(255);
  });
});
