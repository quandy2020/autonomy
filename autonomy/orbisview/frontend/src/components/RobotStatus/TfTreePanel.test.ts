import { describe, expect, it } from 'vitest';
import { buildTfForest, layoutForest } from './TfTreePanel';

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

  it('layouts forest with increasing depth', () => {
    const { roots } = buildTfForest([
      { parent: 'map', child: 'odom', x: 0, y: 0, yaw: 0 },
      { parent: 'odom', child: 'base_link', x: 1, y: 2, yaw: 0.5 },
      { parent: 'base_link', child: 'laser', x: 0.2, y: 0, yaw: 0 },
      { parent: 'base_link', child: 'camera', x: 0.1, y: 0, yaw: 0 },
    ]);
    const { nodes, height } = layoutForest(roots);
    expect(nodes[0].frame).toBe('map');
    expect(nodes[0].children[0].children[0].children).toHaveLength(2);
    expect(height).toBeGreaterThan(200);
  });
});
