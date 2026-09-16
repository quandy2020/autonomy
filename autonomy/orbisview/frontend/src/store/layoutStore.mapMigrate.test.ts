import { describe, expect, it } from 'vitest';
import {
  GROUND_ROBOT_LAYOUT,
  migrateMapLeaves,
  sanitizeMosaic,
  collectMosaicIds,
  insertMosaicLeaf,
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
    expect(node).toMatchObject({
      type: 'split',
      splitPercentages: [70, 30],
    });
  });

  it('sanitize preserves dragged split ratios', () => {
    const node = sanitizeMosaic({
      type: 'split',
      direction: 'row',
      children: [
        'map',
        {
          type: 'split',
          direction: 'column',
          children: ['route', 'dashboard'],
          splitPercentages: [40, 60],
        },
      ],
      splitPercentages: [72, 28],
    });
    expect(node).toMatchObject({
      type: 'split',
      splitPercentages: [72, 28],
      children: [
        'map',
        {
          type: 'split',
          direction: 'column',
          children: ['route', 'dashboard'],
          splitPercentages: [40, 60],
        },
      ],
    });
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

describe('insertMosaicLeaf', () => {
  it('returns leaf when mosaic empty', () => {
    expect(insertMosaicLeaf(null, 'image')).toBe('image');
  });

  it('creates balanced split from a leaf', () => {
    expect(insertMosaicLeaf('map', 'image', 'row')).toEqual({
      type: 'split',
      direction: 'row',
      children: ['map', 'image'],
      splitPercentages: [50, 50],
    });
  });

  it('appends into same-direction n-ary split', () => {
    const cur = insertMosaicLeaf('image', 'image#2', 'row');
    const next = insertMosaicLeaf(cur, 'image#3', 'row');
    expect(next).toMatchObject({
      type: 'split',
      direction: 'row',
      children: ['image', 'image#2', 'image#3'],
    });
    expect(collectMosaicIds(next)).toEqual(['image', 'image#2', 'image#3']);
  });
});
