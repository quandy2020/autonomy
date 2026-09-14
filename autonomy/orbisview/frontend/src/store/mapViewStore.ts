import { create } from 'zustand';
import { persist } from 'zustand/middleware';

export type MapViewMode = '2d' | '3d';
/** Unified nav: 1 point → set_goal；>1 → set_route. pan = drag view. */
export type MapTool = 'pan' | 'measure' | 'nav' | 'pick' | 'poi' | 'draw';

interface MapViewState {
  mode: MapViewMode;
  tool: MapTool;
  statusMsg: string;
  setMode: (mode: MapViewMode) => void;
  setTool: (tool: MapTool) => void;
  setStatusMsg: (msg: string) => void;
  /** Select a map tool; keeps current 2D/3D mode. */
  selectTool: (tool: MapTool) => void;
}

const TOOL_HINT: Record<MapTool, string> = {
  pan: '拖动视图：左键任意方向平移；3D 右键旋转；中键/Shift+左键亦可；滚轮缩放',
  measure: '测距：左键起点，移动预览，右击落终点并显示距离（结果保留至清除/重测）',
  nav: '导航：落点设朝向；1 点=目标，多点=路线，点发送下发；中键/右键/空格+左键拖地图（2D/3D）',
  pick: '取点：左键落点并拖动设朝向，松手复制位置 / yaw / 四元数（2D/3D）',
  poi: 'POI：单击落点；拖移已有点；Delete 删除选中（2D/3D）',
  draw: '绘制：左键加点，双击/右击结束；Esc 取消；Backspace 撤销一点（2D/3D）',
};

function normalizeTool(tool: unknown): MapTool {
  if (tool === 'nav_ab' || tool === 'multi' || tool === 'nav') return 'nav';
  if (
    tool === 'pan' ||
    tool === 'measure' ||
    tool === 'pick' ||
    tool === 'poi' ||
    tool === 'draw'
  ) {
    return tool;
  }
  return 'pan';
}

export const useMapViewStore = create<MapViewState>()(
  persist(
    (set) => ({
      mode: '2d',
      tool: 'pan',
      statusMsg: TOOL_HINT.pan,
      setMode: (mode) => set({ mode }),
      setTool: (tool) => set({ tool, statusMsg: TOOL_HINT[tool] }),
      setStatusMsg: (statusMsg) => set({ statusMsg }),
      selectTool: (tool) =>
        set({
          tool,
          statusMsg: TOOL_HINT[tool],
        }),
    }),
    {
      name: 'orbisview-map-view-v4',
      partialize: (s) => ({ mode: s.mode, tool: s.tool }),
      merge: (persisted, current) => {
        const p = (persisted ?? {}) as Partial<MapViewState> & { tool?: unknown };
        const tool = normalizeTool(p.tool ?? current.tool);
        return {
          ...current,
          ...p,
          tool,
          statusMsg: TOOL_HINT[tool],
        };
      },
    },
  ),
);

export { TOOL_HINT };
