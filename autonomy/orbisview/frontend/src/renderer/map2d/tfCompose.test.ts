import { describe, expect, it } from 'vitest';
import { mergeTfTransforms, resolveTfWorldFrames } from './tfCompose';

describe('resolveTfWorldFrames', () => {
  it('places child links relative to composed base_link, not at raw offsets', () => {
    const frames = resolveTfWorldFrames([
      { parent: 'map', child: 'odom', x: 0, y: 0, yaw: 0 },
      { parent: 'odom', child: 'base_link', x: 3, y: 2, yaw: Math.PI / 2 },
      { parent: 'base_link', child: 'laser_link', x: 0.2, y: 0, yaw: 0 },
      { parent: 'base_link', child: 'wheel_left_link', x: 0, y: 0.15, yaw: 0 },
    ]);
    const by = Object.fromEntries(frames.map((f) => [f.frame, f]));
    expect(by.base_link.x).toBeCloseTo(3, 5);
    expect(by.base_link.y).toBeCloseTo(2, 5);
    // yaw=π/2: local +X → world +Y, local +Y → world -X
    expect(by.laser_link.x).toBeCloseTo(3, 5);
    expect(by.laser_link.y).toBeCloseTo(2.2, 5);
    expect(by.wheel_left_link.x).toBeCloseTo(3 - 0.15, 5);
    expect(by.wheel_left_link.y).toBeCloseTo(2, 5);
  });

  it('does not leave mounts at origin when base_link is elsewhere', () => {
    const frames = resolveTfWorldFrames([
      { parent: 'odom', child: 'base_link', x: -4, y: 1.5, yaw: 0 },
      { parent: 'base_link', child: 'imu_link', x: 0.1, y: 0, yaw: 0 },
    ]);
    const imu = frames.find((f) => f.frame === 'imu_link');
    expect(imu?.x).toBeCloseTo(-3.9, 5);
    expect(imu?.y).toBeCloseTo(1.5, 5);
  });
});

describe('mergeTfTransforms', () => {
  it('lets later list override same child', () => {
    const merged = mergeTfTransforms([
      [{ parent: 'base_link', child: 'laser_link', x: 0, y: 0, yaw: 0 }],
      [{ parent: 'base_link', child: 'laser_link', x: 0.2, y: 0, yaw: 0 }],
    ]);
    expect(merged).toHaveLength(1);
    expect(merged[0].x).toBeCloseTo(0.2);
  });
});
