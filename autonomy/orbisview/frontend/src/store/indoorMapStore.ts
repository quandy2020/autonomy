import { create } from 'zustand';
import { persist } from 'zustand/middleware';
import {
  basemapFromFloor,
  normalizeFloorInfoArray,
  sortFloors,
  type FloorInfoNorm,
} from '@/renderer/map2d/floorBasemap';
import {
  normalizeSemanticZoneArray,
  type SemanticZoneNorm,
} from '@/renderer/map2d/semanticZones';
import { useLayerStore } from '@/store/layoutStore';
import { useMapViewStore } from '@/store/mapViewStore';
import { useMappingVizStore } from '@/store/mappingVizStore';
import { useStaticSlamStore } from '@/store/staticSlamStore';

export type IndoorMapSource = 'none' | 'static' | 'live';

export interface IndoorFixture {
  floors?: unknown;
  zones?: unknown;
  active_floor_id?: string;
  activeFloorId?: string;
}

interface IndoorMapState {
  source: IndoorMapSource;
  floors: FloorInfoNorm[];
  activeFloorId: string | null;
  zones: SemanticZoneNorm[];
  setFromFixture: (f: IndoorFixture) => void;
  applyLiveFloors: (payload: unknown) => void;
  applyLiveZones: (payload: unknown) => void;
  setActiveFloor: (id: string) => boolean;
  stepFloor: (delta: -1 | 1) => boolean;
  clearStatic: () => void;
  applyBasemapForActive: () => void;
}

function applyBasemap(floor: FloorInfoNorm | undefined) {
  if (!floor) return;
  const b = basemapFromFloor(floor);
  if (b) {
    useStaticSlamStore.getState().setBasemap(b);
    useLayerStore.getState().setLayer('basemap', true);
    useMapViewStore.getState().setStatusMsg(`楼层 ${floor.name} · 已加载底图`);
  } else {
    useMapViewStore.getState().setStatusMsg(`楼层 ${floor.name} · 无底图`);
  }
}

export const useIndoorMapStore = create<IndoorMapState>()(
  persist(
    (set, get) => ({
      source: 'none',
      floors: [],
      activeFloorId: null,
      zones: [],
      setFromFixture: (f) => {
        const floorPayload = {
          floors: f.floors ?? [],
          active_floor_id: f.active_floor_id ?? f.activeFloorId,
        };
        const { floors, activeFloorId } = normalizeFloorInfoArray(floorPayload);
        const zones = normalizeSemanticZoneArray(
          Array.isArray(f.zones) ? { zones: f.zones } : f.zones ?? { zones: [] },
        );
        set({ source: 'static', floors, activeFloorId, zones });
        const floor = floors.find((x) => x.id === activeFloorId);
        applyBasemap(floor);
      },
      applyLiveFloors: (payload) => {
        const { floors, activeFloorId } = normalizeFloorInfoArray(payload);
        const prevActive = get().activeFloorId;
        const nextActive =
          activeFloorId && floors.some((x) => x.id === activeFloorId)
            ? activeFloorId
            : prevActive && floors.some((x) => x.id === prevActive)
              ? prevActive
              : floors[0]?.id ?? null;
        set({ source: 'live', floors, activeFloorId: nextActive });
        if (nextActive && nextActive !== prevActive) {
          applyBasemap(floors.find((x) => x.id === nextActive));
        }
      },
      applyLiveZones: (payload) => {
        const zones = normalizeSemanticZoneArray(payload);
        set({ source: 'live', zones });
      },
      setActiveFloor: (id) => {
        if (useMappingVizStore.getState().demo.playing) {
          useMapViewStore.getState().setStatusMsg('请先暂停建图演示');
          return false;
        }
        const { floors } = get();
        if (!floors.some((f) => f.id === id)) return false;
        set({ activeFloorId: id });
        applyBasemap(floors.find((f) => f.id === id));
        return true;
      },
      stepFloor: (delta) => {
        const { floors, activeFloorId } = get();
        const ordered = sortFloors(floors);
        if (!ordered.length) return false;
        const idx = Math.max(
          0,
          ordered.findIndex((f) => f.id === activeFloorId),
        );
        const next = ordered[Math.max(0, Math.min(ordered.length - 1, idx + delta))];
        if (!next || next.id === activeFloorId) return false;
        return get().setActiveFloor(next.id);
      },
      clearStatic: () => {
        if (get().source !== 'static') return;
        set({ source: 'none', floors: [], activeFloorId: null, zones: [] });
      },
      applyBasemapForActive: () => {
        const { floors, activeFloorId } = get();
        applyBasemap(floors.find((f) => f.id === activeFloorId));
      },
    }),
    {
      name: 'orbisview-indoor-map-v1',
      partialize: (s) => ({ activeFloorId: s.activeFloorId }),
      merge: (persisted, current) => {
        const p = persisted as { activeFloorId?: string | null } | undefined;
        return {
          ...current,
          activeFloorId: p?.activeFloorId ?? current.activeFloorId,
        };
      },
    },
  ),
);
