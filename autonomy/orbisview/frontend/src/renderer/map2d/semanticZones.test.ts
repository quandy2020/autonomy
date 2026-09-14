import { describe, expect, it } from 'vitest';
import {
  colorRgbaToCss,
  normalizePolygon,
  normalizeSemanticZone,
  zoneTypePalette,
} from './semanticZones';

describe('normalizePolygon', () => {
  it('accepts object and array points', () => {
    expect(
      normalizePolygon([
        { x: 0, y: 0 },
        [2, 0],
        { x: 2, y: 1 },
        [0, 1],
      ]),
    ).toEqual([
      [0, 0],
      [2, 0],
      [2, 1],
      [0, 1],
    ]);
  });

  it('rejects fewer than 3 points', () => {
    expect(normalizePolygon([[0, 0], [1, 1]])).toBeNull();
  });
});

describe('normalizeSemanticZone', () => {
  it('uses palette when colors missing', () => {
    const z = normalizeSemanticZone({
      id: 'z1',
      zone_type: 'keepout',
      polygon: [
        [0, 0],
        [1, 0],
        [1, 1],
      ],
    });
    expect(z?.fill).toBe(zoneTypePalette('keepout').fill);
    expect(z?.stroke).toBe(zoneTypePalette('keepout').stroke);
  });

  it('applies fill_opacity to wire color', () => {
    const z = normalizeSemanticZone({
      id: 'z2',
      zone_type: 'room',
      polygon: [
        [0, 0],
        [1, 0],
        [0, 1],
      ],
      fill_color: { r: 1, g: 0, b: 0, a: 1 },
      fill_opacity: 0.5,
    });
    expect(z?.fill).toBe('rgba(255,0,0,0.5)');
  });
});

describe('colorRgbaToCss', () => {
  it('handles 0-1 floats', () => {
    expect(colorRgbaToCss({ r: 0, g: 1, b: 0, a: 0.25 })).toBe('rgba(0,255,0,0.25)');
  });
});
