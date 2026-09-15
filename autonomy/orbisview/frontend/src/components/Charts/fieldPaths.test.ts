import { describe, expect, it } from 'vitest';
import { collectNumericPaths, readNumericPath } from './fieldPaths';

describe('fieldPaths', () => {
  it('collects nested scalars and array indices', () => {
    const payload = {
      type: 'ChassisState',
      vx: 0.5,
      wz: -0.1,
      pose: { x: 1, y: 2, yaw: 0.3 },
      wheels: [{ rpm: 10 }, { rpm: 12 }],
      flags: { estop: false },
    };
    const paths = collectNumericPaths(payload);
    expect(paths).toEqual(
      expect.arrayContaining([
        'vx',
        'wz',
        'pose.x',
        'pose.y',
        'pose.yaw',
        'wheels[0].rpm',
        'wheels[1].rpm',
        'flags.estop',
      ]),
    );
    expect(paths).not.toContain('type');
  });

  it('reads numeric and boolean paths', () => {
    const payload = { a: { b: 3.25 }, arr: [9, 8], on: true };
    expect(readNumericPath(payload, 'a.b')).toBe(3.25);
    expect(readNumericPath(payload, 'arr[1]')).toBe(8);
    expect(readNumericPath(payload, 'on')).toBe(1);
    expect(readNumericPath(payload, 'missing')).toBeNull();
  });

  it('skips bulky data keys and indexes number arrays', () => {
    const payload = {
      vx: 1,
      data: new Array(1000).fill(7),
      speed: [0.1, 0.2, 0.3],
    };
    const paths = collectNumericPaths(payload);
    expect(paths).toContain('vx');
    expect(paths).toContain('speed[0]');
    expect(paths).not.toContain('data[0]');
  });

  it('reads twist path aliases', () => {
    const stamped = { vx: 0.4, wz: -0.1, linear: { x: 0.4, y: 0, z: 0 } };
    expect(readNumericPath(stamped, 'twist.linear.x')).toBe(0.4);
    expect(readNumericPath(stamped, 'vx')).toBe(0.4);
    expect(readNumericPath(stamped, 'linear.x')).toBe(0.4);
    const nested = { twist: { linear: { x: 0.5 }, angular: { z: 0.2 } } };
    expect(readNumericPath(nested, 'twist.linear.x')).toBe(0.5);
    expect(readNumericPath(nested, 'linear.x')).toBe(0.5);
  });
});
