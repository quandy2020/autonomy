import { create } from 'zustand';
import { persist } from 'zustand/middleware';

export type MapViewMode = '2d' | '3d';
/** Unified nav: 1 point → set_goal；>1 → set_route. pan = drag view. */
export type MapTool = 'pan' | 'measure' | 'nav' | 'pick' | 'poi' | 'draw';

interface MapViewState {
  mode: MapViewMode;
  tool: MapTool;
  statusMsg: string;
  /** True while a Go-dispatched route/goal is active (Stop clears). */
  routeActive: boolean;
  setMode: (mode: MapViewMode) => void;
  setTool: (tool: MapTool) => void;
  setStatusMsg: (msg: string) => void;
  setRouteActive: (active: boolean) => void;
  /** Select a map tool; keeps current 2D/3D mode. */
  selectTool: (tool: MapTool) => void;
}

const TOOL_HINT: Record<MapTool, string> = {
  pan: '拖动地图 · 滚轮缩放',
  measure: '测距：左键起点 · 右击终点',
  nav: '导航：拖出朝向加点 · 点「出发」开始 · Enter 出发 · Delete 删点 · Esc 清空',
  pick: '取点：拖出朝向 · 松手复制坐标',
  poi: 'POI：单击落点 · Delete 删除',
  draw: '绘制：左键加点 · 双击结束 · Esc 取消',
};

function normalizeTool(tool: unknown): MapTool {
  if (tool === 'nav_ab' || tool === 'multi' || tool === 'nav') return 'nav';
  // Primary map tools only; annotation tools live in POI / Annotations panels.
  if (tool === 'pan' || tool === 'measure') return tool;
  return 'pan';
}

export const useMapViewStore = create<MapViewState>()(
  persist(
    (set) => ({
      mode: '2d',
      tool: 'pan',
      statusMsg: TOOL_HINT.pan,
      routeActive: false,
      setMode: (mode) => set({ mode }),
      setTool: (tool) => set({ tool, statusMsg: TOOL_HINT[tool] }),
      setStatusMsg: (statusMsg) => set({ statusMsg }),
      setRouteActive: (routeActive) => set({ routeActive }),
      selectTool: (tool) =>
        set({
          tool,
          statusMsg: TOOL_HINT[tool],
        }),
    }),
    {
      name: 'orbisview-map-view-v5',
      partialize: (s) => ({ mode: s.mode, tool: s.tool }),
      merge: (persisted, current) => {
        const p = (persisted ?? {}) as Partial<MapViewState> & { tool?: unknown };
        const tool = normalizeTool(p.tool ?? current.tool);
        return {
          ...current,
          ...p,
          tool,
          routeActive: false,
          statusMsg: TOOL_HINT[tool],
        };
      },
    },
  ),
);

export { TOOL_HINT };
