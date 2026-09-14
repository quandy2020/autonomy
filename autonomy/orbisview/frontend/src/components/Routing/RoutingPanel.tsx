import { useMemo } from 'react';
import { useDataStore } from '@/store/dataStore';
import { useWaypointStore } from '@/store/waypointStore';
import { wsClient } from '@/store/websocket/client';
import { SCHEMAS } from '@/store/websocket/types';

export function RoutingPanel() {
  const connected = useDataStore((s) => s.connected);
  const envelopes = useDataStore((s) => s.envelopes);
  const { waypoints, clear } = useWaypointStore();

  const route = useMemo(() => {
    const e = Object.values(envelopes).find((x) => x.schema === SCHEMAS.RoutePath);
    return e?.payload as
      | { state?: string; waypoints?: { x: number; y: number }[] }
      | undefined;
  }, [envelopes]);

  const sendRoute = () => {
    if (!connected || !waypoints.length) return;
    if (waypoints.length === 1) {
      const w = waypoints[0];
      wsClient.send({ op: 'set_goal', x: w.x, y: w.y, yaw: w.yaw ?? 0 });
      return;
    }
    wsClient.send({
      op: 'set_route',
      waypoints: waypoints.map((w) => ({ x: w.x, y: w.y, yaw: w.yaw ?? 0 })),
    });
  };

  const clearRoute = () => {
    clear();
    wsClient.send({ op: 'clear_route' });
    wsClient.send({ op: 'clear_goal' });
  };

  return (
    <div className="panel">
      <h3 style={{ marginTop: 0 }}>Routing</h3>
      <p className="hint">
        1 点 = set_goal · 多点 = set_route（{route?.state ?? '—'}）
      </p>
      <div className="row" style={{ gap: 8, marginBottom: 8, flexWrap: 'wrap' }}>
        <button type="button" disabled={!connected || !waypoints.length} onClick={sendRoute}>
          {waypoints.length <= 1
            ? `set_goal (${waypoints.length})`
            : `set_route (${waypoints.length})`}
        </button>
        <button type="button" disabled={!connected} onClick={clearRoute}>
          clear
        </button>
      </div>
      <ul className="wp-list">
        {(route?.waypoints ?? waypoints).map((w, i) => (
          <li key={i} className="wp-item">
            #{i + 1} ({w.x.toFixed(2)}, {w.y.toFixed(2)})
          </li>
        ))}
      </ul>
    </div>
  );
}
