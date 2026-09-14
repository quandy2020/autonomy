import { describe, expect, it } from 'vitest';
import {
  basemapFromFloor,
  floorHasFetchableSlam,
  normalizeFloorInfo,
  normalizeFloorInfoArray,
  sortFloors,
} from './floorBasemap';

describe('normalizeFloorInfo', () => {
  it('maps strata-like fields', () => {
    const f = normalizeFloorInfo({
      id: 'F1',
      name: '1F',
      level: 1,
      slam_image_path: 'https://example.com/a.png',
      start_x: -10,
      start_y: -10,
      x_grid_count: 200,
      y_grid_count: 100,
      resolution: 0.05,
    });
    expect(f).toMatchObject({
      id: 'F1',
      originX: -10,
      originY: -10,
      widthPx: 200,
      heightPx: 100,
      resolution: 0.05,
    });
  });
});

describe('normalizeFloorInfoArray', () => {
  it('sorts and picks active', () => {
    const { floors, activeFloorId } = normalizeFloorInfoArray({
      floors: [
        { id: 'F2', name: '2F', level: 2, x_grid_count: 10, y_grid_count: 10, resolution: 0.05 },
        { id: 'F1', name: '1F', level: 1, x_grid_count: 10, y_grid_count: 10, resolution: 0.05 },
      ],
      active_floor_id: 'F2',
    });
    expect(floors.map((f) => f.id)).toEqual(['F1', 'F2']);
    expect(activeFloorId).toBe('F2');
  });
});

describe('basemapFromFloor', () => {
  it('builds StaticSlamBasemap for http slam path', () => {
    const f = normalizeFloorInfo({
      id: 'F1',
      name: '1F',
      level: 1,
      slam_image_path: 'https://cdn.example/map.png',
      start_x: -5,
      start_y: -5,
      x_grid_count: 100,
      y_grid_count: 80,
      resolution: 0.05,
    })!;
    expect(floorHasFetchableSlam(f)).toBe(true);
    const b = basemapFromFloor(f);
    expect(b?.imageSrc).toBe('https://cdn.example/map.png');
    expect(b?.originX).toBe(-5);
    expect(b?.widthPx).toBe(100);
  });

  it('returns null without slam path', () => {
    const f = normalizeFloorInfo({
      id: 'F1',
      name: '1F',
      level: 1,
      x_grid_count: 10,
      y_grid_count: 10,
      resolution: 0.05,
    })!;
    expect(basemapFromFloor(f)).toBeNull();
  });
});

describe('sortFloors', () => {
  it('orders by level', () => {
    expect(
      sortFloors([
        { id: 'b', name: 'B', level: 2, originX: 0, originY: 0, widthPx: 1, heightPx: 1, resolution: 0.05 },
        { id: 'a', name: 'A', level: 0, originX: 0, originY: 0, widthPx: 1, heightPx: 1, resolution: 0.05 },
      ]).map((f) => f.id),
    ).toEqual(['a', 'b']);
  });
});
