import { describe, expect, it } from 'vitest';
import { formatPickPose, yawToDeg, yawToQuaternion } from './poseMath';

describe('poseMath', () => {
  it('maps yaw 0 to identity-like quat', () => {
    const q = yawToQuaternion(0);
    expect(q.x).toBeCloseTo(0);
    expect(q.y).toBeCloseTo(0);
    expect(q.z).toBeCloseTo(0);
    expect(q.w).toBeCloseTo(1);
  });

  it('maps yaw π/2 around Z', () => {
    const q = yawToQuaternion(Math.PI / 2);
    expect(q.x).toBeCloseTo(0);
    expect(q.y).toBeCloseTo(0);
    expect(q.z).toBeCloseTo(Math.SQRT1_2);
    expect(q.w).toBeCloseTo(Math.SQRT1_2);
    expect(yawToDeg(Math.PI / 2)).toBeCloseTo(90);
  });

  it('formats pick clipboard text', () => {
    const text = formatPickPose(1, 2, Math.PI / 2);
    expect(text).toContain('x=1.0000 y=2.0000');
    expect(text).toContain('yaw=');
    expect(text).toContain('qx=');
    expect(text).toContain('qw=');
  });
});
