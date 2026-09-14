import { Map2DPanel } from '@/components/Map2D/Map2DPanel';
import { View3DPanel } from '@/components/View3D/View3DPanel';
import type { PanelProps } from '@/components/registry';
import { useMapViewStore } from '@/store/mapViewStore';

/** Unified map workspace: keep both views mounted so 2D↔3D switch stays warm. */
export function MapPanel(_props: PanelProps) {
  const mode = useMapViewStore((s) => s.mode);

  return (
    <div className="panel map-panel">
      <div className="map-stage">
        <div
          className={`map-mode-pane${mode === '2d' ? ' is-active' : ''}`}
          aria-hidden={mode !== '2d'}
        >
          <Map2DPanel />
        </div>
        <div
          className={`map-mode-pane${mode === '3d' ? ' is-active' : ''}`}
          aria-hidden={mode !== '3d'}
        >
          <View3DPanel active={mode === '3d'} />
        </div>
      </div>
    </div>
  );
}
