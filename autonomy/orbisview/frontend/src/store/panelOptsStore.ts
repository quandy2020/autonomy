import { create } from 'zustand';
import { persist } from 'zustand/middleware';

export interface ImagePanelOpts {
  channel: string | null;
}

export interface ChartSeriesSpec {
  id: string;
  channel: string;
  field: string;
  color: string;
  enabled: boolean;
}

export interface ChartPanelOpts {
  series: ChartSeriesSpec[];
  windowMs: number;
}

export const CHART_SERIES_COLORS = [
  '#4fc3f7',
  '#69f0ae',
  '#ffd54f',
  '#ce93d8',
  '#ef9a9a',
  '#ffcc80',
  '#80cbc4',
  '#f48fb1',
  '#90caf9',
  '#a5d6a7',
] as const;

export const DEFAULT_CHART_WINDOW_MS = 15_000;

export function defaultChartOpts(): ChartPanelOpts {
  return { series: [], windowMs: DEFAULT_CHART_WINDOW_MS };
}

export interface PanelOptsState {
  image: Record<string, ImagePanelOpts>;
  charts: Record<string, ChartPanelOpts>;
  setImageChannel: (panelId: string, channel: string | null) => void;
  setChartWindowMs: (panelId: string, windowMs: number) => void;
  addChartSeries: (
    panelId: string,
    channel: string,
    field: string,
    color?: string,
  ) => string;
  removeChartSeries: (panelId: string, seriesId: string) => void;
  setChartSeriesEnabled: (panelId: string, seriesId: string, enabled: boolean) => void;
  setChartSeriesColor: (panelId: string, seriesId: string, color: string) => void;
  clearChartSeries: (panelId: string) => void;
  prunePanels: (aliveIds: string[]) => void;
}

function chartOpts(s: PanelOptsState, panelId: string): ChartPanelOpts {
  return (s.charts ?? {})[panelId] ?? defaultChartOpts();
}

export const usePanelOptsStore = create<PanelOptsState>()(
  persist(
    (set, get) => ({
      image: {},
      charts: {},
      setImageChannel: (panelId, channel) =>
        set((s) => ({
          image: {
            ...s.image,
            [panelId]: { channel },
          },
        })),
      setChartWindowMs: (panelId, windowMs) =>
        set((s) => ({
          charts: {
            ...s.charts,
            [panelId]: { ...chartOpts(s, panelId), windowMs },
          },
        })),
      addChartSeries: (panelId, channel, field, color) => {
        const id = `cs-${Date.now()}-${Math.random().toString(36).slice(2, 6)}`;
        const cur = chartOpts(get(), panelId);
        const nextColor =
          color ?? CHART_SERIES_COLORS[cur.series.length % CHART_SERIES_COLORS.length]!;
        const spec: ChartSeriesSpec = {
          id,
          channel,
          field,
          color: nextColor,
          enabled: true,
        };
        set((s) => ({
          charts: {
            ...s.charts,
            [panelId]: { ...chartOpts(s, panelId), series: [...cur.series, spec] },
          },
        }));
        return id;
      },
      removeChartSeries: (panelId, seriesId) =>
        set((s) => {
          const cur = chartOpts(s, panelId);
          return {
            charts: {
              ...s.charts,
              [panelId]: {
                ...cur,
                series: cur.series.filter((x) => x.id !== seriesId),
              },
            },
          };
        }),
      setChartSeriesEnabled: (panelId, seriesId, enabled) =>
        set((s) => {
          const cur = chartOpts(s, panelId);
          return {
            charts: {
              ...s.charts,
              [panelId]: {
                ...cur,
                series: cur.series.map((x) => (x.id === seriesId ? { ...x, enabled } : x)),
              },
            },
          };
        }),
      setChartSeriesColor: (panelId, seriesId, color) =>
        set((s) => {
          const cur = chartOpts(s, panelId);
          return {
            charts: {
              ...s.charts,
              [panelId]: {
                ...cur,
                series: cur.series.map((x) => (x.id === seriesId ? { ...x, color } : x)),
              },
            },
          };
        }),
      clearChartSeries: (panelId) =>
        set((s) => ({
          charts: {
            ...s.charts,
            [panelId]: { ...chartOpts(s, panelId), series: [] },
          },
        })),
      prunePanels: (aliveIds) =>
        set((s) => {
          const alive = new Set(aliveIds);
          const image: Record<string, ImagePanelOpts> = {};
          for (const [id, opts] of Object.entries(s.image ?? {})) {
            if (alive.has(id)) image[id] = opts;
          }
          const charts: Record<string, ChartPanelOpts> = {};
          for (const [id, opts] of Object.entries(s.charts ?? {})) {
            if (alive.has(id)) charts[id] = opts;
          }
          return { image, charts };
        }),
    }),
    {
      name: 'orbisview-panel-opts-v1',
      partialize: (s) => ({ image: s.image, charts: s.charts }),
      merge: (persisted, current) => {
        const p = (persisted ?? {}) as Partial<Pick<PanelOptsState, 'image' | 'charts'>>;
        return {
          ...current,
          image: p.image ?? {},
          charts: p.charts ?? {},
        };
      },
    },
  ),
);
