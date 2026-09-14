import { describe, expect, it } from 'vitest';
import {
  invertSe2,
  lookupTransform,
  mergeTfTransforms,
  resolveLaserDrawPose,
  resolveTfWorldFrames,
  transformPointSe2,
} from './tfCompose';

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

describe('resolveLaserDrawPose', () => {
  it('prefers laser_link over base_link', () => {
    const frames = resolveTfWorldFrames([
      { parent: 'map', child: 'odom', x: 0, y: 0, yaw: 0 },
      { parent: 'odom', child: 'base_link', x: 1, y: 1, yaw: 0 },
      { parent: 'base_link', child: 'laser_link', x: 0.2, y: 0, yaw: 0 },
    ]);
    const laser = resolveLaserDrawPose(frames, 'laser_link', { x: 0, y: 0, yaw: 0 });
    expect(laser?.x).toBeCloseTo(1.2, 5);
    expect(laser?.y).toBeCloseTo(1, 5);
  });
});

describe('lookupTransform (autoviz-style)', () => {
  const tree = [
    { parent: 'map', child: 'odom', x: 1, y: 0, yaw: 0 },
    { parent: 'odom', child: 'base_link', x: 3, y: 2, yaw: Math.PI / 2 },
    { parent: 'base_link', child: 'laser_link', x: 0.2, y: 0, yaw: 0 },
  ];

  it('returns pose of laser_link in map', () => {
    const tf = lookupTransform(tree, 'map', 'laser_link');
    expect(tf).not.toBeNull();
    // base at (1+3, 2)=(4,2) yaw π/2; laser local +0.2 x → world +0.2 y
    expect(tf!.x).toBeCloseTo(4, 5);
    expect(tf!.y).toBeCloseTo(2.2, 5);
    expect(tf!.yaw).toBeCloseTo(Math.PI / 2, 5);
  });

  it('transforms local scan point into map like Autoviz', () => {
    const tf = lookupTransform(tree, 'map', 'laser_link')!;
    // local forward 1m at angle 0 → laser +X; with yaw π/2 → map +Y
    const p = transformPointSe2(tf, 1, 0);
    expect(p.x).toBeCloseTo(4, 5);
    expect(p.y).toBeCloseTo(3.2, 5);
  });

  it('returns null when map and odom are disconnected (no bad fallback)', () => {
    const partial = [
      { parent: 'odom', child: 'base_link', x: 1, y: 0, yaw: 0 },
      { parent: 'base_link', child: 'laser_link', x: 0.2, y: 0, yaw: 0 },
    ];
    expect(lookupTransform(partial, 'map', 'laser_link')).toBeNull();
  });

  it('invertSe2 undoes translation of lookup', () => {
    const tf = lookupTransform(tree, 'map', 'laser_link')!;
    const origin = transformPointSe2(invertSe2(tf), tf.x, tf.y);
    expect(origin.x).toBeCloseTo(0, 5);
    expect(origin.y).toBeCloseTo(0, 5);
  });
});
