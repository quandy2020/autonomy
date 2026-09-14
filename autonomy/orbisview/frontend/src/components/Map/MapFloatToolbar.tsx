import { Icon, type IconName } from '@/components/icons';
import { MapViewModeToggle } from '@/components/Map/MapViewModeToggle';
import { StaticSlamLoadPop } from '@/components/Map/StaticSlamLoadPop';
import { useMapViewStore, type MapTool } from '@/store/mapViewStore';
import { useLayerStore } from '@/store/layoutStore';
import { useWaypointStore } from '@/store/waypointStore';
import { useDataStore } from '@/store/dataStore';
import { useStaticSlamStore } from '@/store/staticSlamStore';
import { useMappingVizStore } from '@/store/mappingVizStore';
import { wsClient } from '@/store/websocket/client';
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

const TOOL_META: Record<MapTool, { icon: IconName; label: string }> = {
  pan: { icon: 'pan', label: '拖动视图' },
  measure: { icon: 'measure', label: '测距' },
  nav: { icon: 'nav', label: '导航' },
  pick: { icon: 'pick', label: '取点' },
};

function RailBtn({
  title,
  active,
  disabled,
  onClick,
  icon,
  badge,
}: {
  title: string;
  active?: boolean;
  disabled?: boolean;
  onClick: () => void;
  icon: IconName;
  badge?: string | number;
}) {
  return (
    <button
      type="button"
      title={title}
      aria-label={title}
      aria-pressed={active}
      className={active ? 'map-rail-btn active' : 'map-rail-btn'}
      disabled={disabled}
      onClick={onClick}
    >
      <Icon name={icon} size={16} />
      {badge != null && badge !== '' ? <span className="map-rail-badge">{badge}</span> : null}
    </button>
  );
}

/** Right-side vertical floating tool rail for Map 2D / 3D. */
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
  const clearWaypoints = useWaypointStore((s) => s.clear);
  const connected = useDataStore((s) => s.connected);

  const sendNav = () => {
    if (!connected || !waypoints.length) return;
    if (waypoints.length === 1) {
      const wp = waypoints[0];
      wsClient.send({ op: 'set_goal', x: wp.x, y: wp.y, yaw: wp.yaw ?? 0 });
      useMapViewStore.getState().setStatusMsg(
        `已发送目标 (${wp.x.toFixed(2)}, ${wp.y.toFixed(2)})`,
      );
      return;
    }
    wsClient.send({
      op: 'set_route',
      waypoints: waypoints.map((wp) => ({
        x: wp.x,
        y: wp.y,
        yaw: wp.yaw ?? 0,
      })),
    });
    useMapViewStore.getState().setStatusMsg(`已发送路线 ${waypoints.length} 点`);
  };

  const clearNav = () => {
    clearWaypoints();
    wsClient.send({ op: 'clear_route' });
    wsClient.send({ op: 'clear_goal' });
    onClearGoal?.();
    useMapViewStore.getState().setStatusMsg('已清空导航点');
  };

  const sendTitle =
    waypoints.length <= 1
      ? `发送目标 (${waypoints.length})`
      : `发送路线 (${waypoints.length})`;

  return (
    <>
      <div className="map-float-rail" role="toolbar" aria-label="Map tools">
        <MapViewModeToggle />

        <span className="map-rail-sep" />

        <div className="map-rail-group" aria-label="Interaction tools">
          {(Object.keys(TOOL_META) as MapTool[]).map((id) => (
            <RailBtn
              key={id}
              title={TOOL_META[id].label}
              icon={TOOL_META[id].icon}
              active={tool === id}
              onClick={() => selectTool(id)}
            />
          ))}
          <RailBtn
            title={sendTitle}
            icon="send"
            disabled={!connected || !waypoints.length}
            onClick={sendNav}
            badge={waypoints.length || undefined}
          />
          <RailBtn
            title="一键清除导航点"
            icon="clear"
            disabled={!waypoints.length && !goalLabel}
            onClick={clearNav}
          />
          {tool === 'measure' && measureActive ? (
            <RailBtn title="清除测距" icon="stop" onClick={() => onClearMeasure?.()} />
          ) : null}
        </div>

        <span className="map-rail-sep" />

        <div className="map-rail-group" aria-label="View controls">
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
            title={
              demoPlaying
                ? '建图演示播放中 — 请先暂停'
                : '静态底图'
            }
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
