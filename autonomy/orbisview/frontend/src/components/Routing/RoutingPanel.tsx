import { useMemo } from 'react';
import { useDataStore } from '@/store/dataStore';
import { useWaypointStore } from '@/store/waypointStore';
import { goNavigation, stopNavigation } from '@/store/navActions';
import { SCHEMAS } from '@/store/websocket/types';

/** Route status mirror — editing happens on Map. */
export function RoutingPanel() {
  const connected = useDataStore((s) => s.connected);
  const envelopes = useDataStore((s) => s.envelopes);
  const waypoints = useWaypointStore((s) => s.waypoints);

  const route = useMemo(() => {
    const e = Object.values(envelopes).find((x) => x.schema === SCHEMAS.RoutePath);
    return e?.payload as
      | { state?: string; waypoints?: { x: number; y: number }[] }
      | undefined;
  }, [envelopes]);

  const n = waypoints.length;
  const shown = route?.waypoints ?? waypoints;

  return (
    <div className="panel">
      <h3 style={{ marginTop: 0 }}>路线</h3>
      <p className="hint">
        状态 {route?.state ?? '空闲'} · 在 Map 加点后「出发」
      </p>
      <div className="row" style={{ gap: 8, marginBottom: 8, flexWrap: 'wrap' }}>
        <button
          type="button"
          className="btn-primary"
          disabled={!connected || !n}
          onClick={() => goNavigation()}
        >
          {n <= 1 ? '出发' : `出发 (${n})`}
        </button>
        <button type="button" onClick={() => stopNavigation(true)}>
          停止清空
        </button>
      </div>
      <ul className="wp-list">
        {shown.map((w, i) => (
          <li key={i} className="wp-item">
            {i + 1}. ({w.x.toFixed(2)}, {w.y.toFixed(2)})
          </li>
        ))}
      </ul>
      {!shown.length ? <p className="muted">暂无路线</p> : null}
    </div>
  );
}
