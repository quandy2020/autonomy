import { describe, expect, it } from 'vitest';
import {
  parseStaticSlamSidecar,
  staticSlamCorners,
  staticSlamWorldSize,
} from './staticSlam';

describe('parseStaticSlamSidecar', () => {
  it('parses Nav2-style YAML', () => {
    const text = `
image: turtlebot3_world.pgm
resolution: 0.050000
origin: [-10.000000, -10.000000, 0.000000]
negate: 0
`;
    const m = parseStaticSlamSidecar(text, 'yaml');
    expect(m.image).toBe('turtlebot3_world.pgm');
    expect(m.resolution).toBeCloseTo(0.05);
    expect(m.originX).toBeCloseTo(-10);
    expect(m.originY).toBeCloseTo(-10);
  });

  it('parses BICMap-style JSON aliases', () => {
    const m = parseStaticSlamSidecar(
      JSON.stringify({
        startX: -58.99,
        startY: -21.35,
        resolution: 0.05,
        xGridCount: 100,
        yGridCount: 50,
        image: 'slam.png',
      }),
      'json',
    );
    expect(m.originX).toBeCloseTo(-58.99);
    expect(m.originY).toBeCloseTo(-21.35);
    expect(m.widthPx).toBe(100);
    expect(m.heightPx).toBe(50);
    expect(m.image).toBe('slam.png');
  });
});

describe('staticSlamCorners', () => {
  it('builds AABB from origin and pixel size', () => {
    expect(
      staticSlamCorners({
        originX: -10,
        originY: -10,
        widthPx: 200,
        heightPx: 200,
        resolution: 0.05,
      }),
    ).toEqual({ x0: -10, y0: -10, x1: 0, y1: 0 });
    expect(staticSlamWorldSize({ widthPx: 200, heightPx: 200, resolution: 0.05 })).toEqual({
      worldW: 10,
      worldH: 10,
    });
  });
});
