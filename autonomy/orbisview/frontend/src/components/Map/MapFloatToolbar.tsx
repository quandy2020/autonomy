import { Icon, type IconName } from '@/components/icons';
import { MapViewModeToggle } from '@/components/Map/MapViewModeToggle';
import { StaticSlamLoadPop } from '@/components/Map/StaticSlamLoadPop';
import { useMapViewStore, type MapTool } from '@/store/mapViewStore';
import { useLayerStore } from '@/store/layoutStore';
import { useWaypointStore } from '@/store/waypointStore';
import { useDataStore } from '@/store/dataStore';
import { useStaticSlamStore } from '@/store/staticSlamStore';
import { useMappingVizStore } from '@/store/mappingVizStore';
import { goNavigation, stopNavigation } from '@/store/navActions';
import { useState } from 'react';

interface Props {
  onClearMeasure?: () => void;
  measureActive?: boolean;
  goalLabel?: string;
  onClearGoal?: () => void;
  onZoomIn?: () => void;
  onZoomOut?: () => void;
  onFit?: () => void;
}

/** Primary map tools only — annotations live in their own panel. */
const PRIMARY_TOOLS: MapTool[] = ['pan', 'nav', 'measure'];

const TOOL_META: Record<MapTool, { icon: IconName; label: string }> = {
  pan: { icon: 'pan', label: '拖动' },
  measure: { icon: 'measure', label: '测距' },
  nav: { icon: 'nav', label: '导航' },
  pick: { icon: 'pick', label: '取点' },
  poi: { icon: 'waypoint', label: 'POI' },
  draw: { icon: 'layers', label: '绘制' },
};

function RailBtn({
  title,
  active,
  disabled,
  onClick,
  icon,
  badge,
  tone,
}: {
  title: string;
  active?: boolean;
  disabled?: boolean;
  onClick: () => void;
  icon: IconName;
  badge?: string | number;
  tone?: 'go' | 'stop';
}) {
  const toneCls = tone === 'go' ? ' map-rail-btn--go' : tone === 'stop' ? ' map-rail-btn--stop' : '';
  return (
    <button
      type="button"
      title={title}
      aria-label={title}
      aria-pressed={active}
      className={`map-rail-btn${active ? ' active' : ''}${toneCls}`}
      disabled={disabled}
      onClick={onClick}
    >
      <Icon name={icon} size={16} />
      {badge != null && badge !== '' ? <span className="map-rail-badge">{badge}</span> : null}
    </button>
  );
}

/** Right-side floating tool rail — nav product chrome. */
export function MapFloatToolbar({
  onClearMeasure,
  measureActive,
  goalLabel,
  onClearGoal,
  onZoomIn,
  onZoomOut,
  onFit,
}: Props) {
  const [basemapOpen, setBasemapOpen] = useState(false);
  const tool = useMapViewStore((s) => s.tool);
  const statusMsg = useMapViewStore((s) => s.statusMsg);
  const selectTool = useMapViewStore((s) => s.selectTool);
  const followRobot = useLayerStore((s) => s.followRobot);
  const setFollowRobot = useLayerStore((s) => s.setFollowRobot);
  const hasBasemap = !!useStaticSlamStore((s) => s.basemap);
  const demoPlaying = useMappingVizStore((s) => s.demo.playing);
  const waypoints = useWaypointStore((s) => s.waypoints);
  const connected = useDataStore((s) => s.connected);

  const n = waypoints.length;
  const canGo = connected && n > 0;
  const canStop = n > 0 || !!goalLabel;

  const onGo = () => {
    goNavigation();
  };

  const onStop = () => {
    stopNavigation(true);
    // Local UI only — stopNavigation already published /cancel_navigation.
    onClearGoal?.();
  };

  return (
    <>
      <div className="map-float-rail" role="toolbar" aria-label="地图工具">
        <MapViewModeToggle />

        <span className="map-rail-sep" />

        <div className="map-rail-group" aria-label="工具">
          {PRIMARY_TOOLS.map((id) => (
            <RailBtn
              key={id}
              title={TOOL_META[id].label}
              icon={TOOL_META[id].icon}
              active={tool === id}
              onClick={() => selectTool(id)}
            />
          ))}
          {tool === 'measure' && measureActive ? (
            <RailBtn title="清除测距" icon="clear" onClick={() => onClearMeasure?.()} />
          ) : null}
        </div>

        <span className="map-rail-sep" />

        <div className="map-rail-group" aria-label="导航">
          <RailBtn
            title={n <= 1 ? `出发${n ? ` (${n}点)` : ''}` : `出发 (${n}点)`}
            icon="send"
            tone="go"
            disabled={!canGo}
            onClick={onGo}
            badge={n || undefined}
          />
          <RailBtn
            title="停止并清空"
            icon="stop"
            tone="stop"
            disabled={!canStop}
            onClick={onStop}
          />
        </div>

        <span className="map-rail-sep" />

        <div className="map-rail-group" aria-label="视图">
          <RailBtn title="放大" icon="zoomIn" onClick={() => onZoomIn?.()} disabled={!onZoomIn} />
          <RailBtn title="缩小" icon="zoomOut" onClick={() => onZoomOut?.()} disabled={!onZoomOut} />
          <RailBtn title="自适应" icon="fit" onClick={() => onFit?.()} disabled={!onFit} />
          <RailBtn
            title={followRobot ? '跟随中' : '跟随机器人'}
            icon="follow"
            active={followRobot}
            onClick={() => setFollowRobot(!followRobot)}
          />
          <RailBtn
            title={demoPlaying ? '建图演示中' : '静态底图'}
            icon="mapping"
            active={basemapOpen || hasBasemap}
            disabled={demoPlaying}
            onClick={() => {
              if (demoPlaying) return;
              setBasemapOpen((v) => !v);
            }}
          />
        </div>
      </div>

      {basemapOpen ? (
        <div className="map-basemap-anchor">
          <StaticSlamLoadPop onClose={() => setBasemapOpen(false)} />
        </div>
      ) : null}

      <div className="map-float-status muted" title={statusMsg}>
        {statusMsg}
        {goalLabel ? ` · ${goalLabel}` : ''}
      </div>
    </>
  );
}
