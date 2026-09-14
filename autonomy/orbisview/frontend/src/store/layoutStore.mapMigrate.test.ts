import { describe, expect, it } from 'vitest';
import {
  GROUND_ROBOT_LAYOUT,
  migrateMapLeaves,
  sanitizeMosaic,
  collectMosaicIds,
} from '@/store/layoutStore';

describe('migrateMapLeaves', () => {
  it('rewrites map2d and view3d to map', () => {
    expect(migrateMapLeaves('map2d')).toBe('map');
    expect(migrateMapLeaves('view3d')).toBe('map');
    expect(migrateMapLeaves('dashboard')).toBe('dashboard');
  });

  it('sanitize keeps single map after migration', () => {
    const node = sanitizeMosaic({
      type: 'split',
      direction: 'row',
      children: ['map2d', 'dashboard'],
      splitPercentages: [70, 30],
    });
    expect(collectMosaicIds(node)).toEqual(['map', 'dashboard']);
  });

  it('sanitize resets when both map2d and view3d collapse to duplicate map', () => {
    const node = sanitizeMosaic({
      type: 'split',
      direction: 'row',
      children: ['map2d', 'view3d'],
      splitPercentages: [50, 50],
    });
    expect(node).toEqual(GROUND_ROBOT_LAYOUT);
  });
});
