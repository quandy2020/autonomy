import { create } from 'zustand';
import { persist } from 'zustand/middleware';
import {
  MAX_POIS,
  MAX_SHAPE_POINTS,
  newAnnotationId,
  parseAnnotationFixture,
  serializeAnnotations,
  type DrawShapeKind,
  type MapDrawShape,
  type MapPoi,
  type PoiKind,
} from '@/renderer/map2d/annotations';

export type AnnotationDraft = {
  tool: 'draw';
  kind: DrawShapeKind;
  points: [number, number][];
};

interface AnnotationState {
  pois: MapPoi[];
  shapes: MapDrawShape[];
  draft: AnnotationDraft | null;
  draftPreview: { x: number; y: number } | null;
  poiDefaultKind: PoiKind;
  drawDefaultKind: DrawShapeKind;
  selectedId: string | null;
  setPoiDefaultKind: (k: PoiKind) => void;
  setDrawDefaultKind: (k: DrawShapeKind) => void;
  setSelected: (id: string | null) => void;
  setDraftPreview: (p: { x: number; y: number } | null) => void;
  addPoi: (p: Omit<MapPoi, 'id'> & { id?: string }) => boolean;
  updatePoi: (id: string, patch: Partial<MapPoi>) => void;
  removePoi: (id: string) => void;
  addShape: (s: Omit<MapDrawShape, 'id'> & { id?: string }) => boolean;
  updateShape: (id: string, patch: Partial<MapDrawShape>) => void;
  removeShape: (id: string) => void;
  ensureDraft: () => void;
  appendDraftPoint: (x: number, y: number) => void;
  undoDraftPoint: () => void;
  commitDraft: () => boolean;
  cancelDraft: () => void;
  importJson: (text: string) => void;
  exportJson: () => string;
  clearAll: () => void;
}

export const useAnnotationStore = create<AnnotationState>()(
  persist(
    (set, get) => ({
      pois: [],
      shapes: [],
      draft: null,
      draftPreview: null,
      poiDefaultKind: 'charger',
      drawDefaultKind: 'polygon',
      selectedId: null,
      setPoiDefaultKind: (k) => set({ poiDefaultKind: k }),
      setDrawDefaultKind: (k) => set({ drawDefaultKind: k }),
      setSelected: (id) => set({ selectedId: id }),
      setDraftPreview: (p) => set({ draftPreview: p }),
      addPoi: (p) => {
        const { pois, poiDefaultKind } = get();
        if (pois.length >= MAX_POIS) return false;
        const next: MapPoi = {
          id: p.id ?? newAnnotationId(),
          x: p.x,
          y: p.y,
          yaw: p.yaw,
          kind: p.kind ?? poiDefaultKind,
          label: p.label,
          color: p.color,
        };
        set({ pois: [...pois, next], selectedId: next.id });
        return true;
      },
      updatePoi: (id, patch) =>
        set({
          pois: get().pois.map((p) => (p.id === id ? { ...p, ...patch, id: p.id } : p)),
        }),
      removePoi: (id) =>
        set({
          pois: get().pois.filter((p) => p.id !== id),
          selectedId: get().selectedId === id ? null : get().selectedId,
        }),
      addShape: (s) => {
        const points = s.points.slice(0, MAX_SHAPE_POINTS);
        const min = s.kind === 'polygon' ? 3 : 2;
        if (points.length < min) return false;
        const next: MapDrawShape = {
          id: s.id ?? newAnnotationId(),
          kind: s.kind,
          points,
          label: s.label,
          stroke: s.stroke,
          fill: s.fill,
        };
        set({ shapes: [...get().shapes, next], selectedId: next.id });
        return true;
      },
      updateShape: (id, patch) =>
        set({
          shapes: get().shapes.map((s) =>
            s.id === id ? { ...s, ...patch, id: s.id } : s,
          ),
        }),
      removeShape: (id) =>
        set({
          shapes: get().shapes.filter((s) => s.id !== id),
          selectedId: get().selectedId === id ? null : get().selectedId,
        }),
      ensureDraft: () => {
        if (get().draft) return;
        set({
          draft: { tool: 'draw', kind: get().drawDefaultKind, points: [] },
        });
      },
      appendDraftPoint: (x, y) => {
        get().ensureDraft();
        const draft = get().draft;
        if (!draft) return;
        if (draft.points.length >= MAX_SHAPE_POINTS) return;
        set({
          draft: { ...draft, points: [...draft.points, [x, y]] },
        });
      },
      undoDraftPoint: () => {
        const draft = get().draft;
        if (!draft || !draft.points.length) return;
        set({ draft: { ...draft, points: draft.points.slice(0, -1) } });
      },
      commitDraft: () => {
        const draft = get().draft;
        if (!draft) return false;
        const ok = get().addShape({ kind: draft.kind, points: draft.points });
        if (ok) set({ draft: null, draftPreview: null });
        return ok;
      },
      cancelDraft: () => set({ draft: null, draftPreview: null }),
      importJson: (text) => {
        const { pois, shapes } = parseAnnotationFixture(text);
        set({ pois, shapes, draft: null, draftPreview: null, selectedId: null });
      },
      exportJson: () => serializeAnnotations(get().pois, get().shapes),
      clearAll: () =>
        set({
          pois: [],
          shapes: [],
          draft: null,
          draftPreview: null,
          selectedId: null,
        }),
    }),
    {
      name: 'orbisview-annotations-v1',
      partialize: (s) => ({
        pois: s.pois,
        shapes: s.shapes,
        poiDefaultKind: s.poiDefaultKind,
        drawDefaultKind: s.drawDefaultKind,
      }),
    },
  ),
);
