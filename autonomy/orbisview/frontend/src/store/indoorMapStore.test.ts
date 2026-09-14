import { beforeEach, describe, expect, it, vi } from 'vitest';

const setBasemap = vi.fn();
const setLayer = vi.fn();
const setStatusMsg = vi.fn();
const mappingState = { demo: { playing: false } };

vi.mock('@/store/staticSlamStore', () => ({
  useStaticSlamStore: {
    getState: () => ({ setBasemap }),
  },
}));

vi.mock('@/store/layoutStore', () => ({
  useLayerStore: {
    getState: () => ({ setLayer }),
  },
}));

vi.mock('@/store/mapViewStore', () => ({
  useMapViewStore: {
    getState: () => ({ setStatusMsg }),
  },
}));

vi.mock('@/store/mappingVizStore', () => ({
  useMappingVizStore: {
    getState: () => mappingState,
  },
}));

import { useIndoorMapStore } from './indoorMapStore';

beforeEach(() => {
  setBasemap.mockClear();
  setLayer.mockClear();
  setStatusMsg.mockClear();
  mappingState.demo.playing = false;
  useIndoorMapStore.setState({
    source: 'none',
    floors: [],
    activeFloorId: null,
    zones: [],
  });
});

describe('setFromFixture', () => {
  it('loads floors and zones', () => {
    useIndoorMapStore.getState().setFromFixture({
      floors: [
        {
          id: 'F1',
          name: '1F',
          level: 1,
          slam_image_path: 'https://example.com/a.png',
          start_x: -10,
          start_y: -10,
          x_grid_count: 100,
          y_grid_count: 100,
          resolution: 0.05,
        },
      ],
      active_floor_id: 'F1',
      zones: [
        {
          id: 'z1',
          zone_type: 'keepout',
          polygon: [
            [0, 0],
            [1, 0],
            [1, 1],
          ],
        },
      ],
    });
    const s = useIndoorMapStore.getState();
    expect(s.source).toBe('static');
    expect(s.floors).toHaveLength(1);
    expect(s.zones).toHaveLength(1);
    expect(s.activeFloorId).toBe('F1');
    expect(setBasemap).toHaveBeenCalled();
  });
});

describe('setActiveFloor', () => {
  it('blocks while mapping demo plays', () => {
    useIndoorMapStore.setState({
      floors: [
        {
          id: 'F1',
          name: '1F',
          level: 1,
          originX: 0,
          originY: 0,
          widthPx: 10,
          heightPx: 10,
          resolution: 0.05,
        },
      ],
      activeFloorId: 'F1',
    });
    mappingState.demo.playing = true;
    expect(useIndoorMapStore.getState().setActiveFloor('F1')).toBe(false);
    expect(setStatusMsg).toHaveBeenCalled();
  });
});
