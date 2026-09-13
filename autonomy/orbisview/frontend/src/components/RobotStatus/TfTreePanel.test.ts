import { describe, expect, it } from 'vitest';
import { buildTfForest } from './TfTreePanel';

describe('buildTfForest', () => {
  it('builds map → odom → base_link hierarchy', () => {
    const { roots, cycles, orphans } = buildTfForest([
      { parent: 'map', child: 'odom', x: 0, y: 0, yaw: 0 },
      { parent: 'odom', child: 'base_link', x: -3, y: -2.7, yaw: 2.4 },
    ]);
    expect(cycles).toEqual([]);
    expect(orphans).toEqual([]);
    expect(roots).toHaveLength(1);
    expect(roots[0].frame).toBe('map');
    expect(roots[0].children[0].frame).toBe('odom');
    expect(roots[0].children[0].children[0].frame).toBe('base_link');
    expect(roots[0].children[0].children[0].edge?.x).toBe(-3);
  });

  it('supports branching children', () => {
    const { roots } = buildTfForest([
      { parent: 'base_link', child: 'laser', x: 0.2, y: 0, yaw: 0 },
      { parent: 'base_link', child: 'camera', x: 0.1, y: 0, yaw: 0 },
    ]);
    expect(roots[0].frame).toBe('base_link');
    expect(roots[0].children.map((c) => c.frame).sort()).toEqual(['camera', 'laser']);
  });
});
