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
    wsClient.send({
      op: 'set_route',
      waypoints: waypoints.map((w) => ({ x: w.x, y: w.y })),
    });
  };

  const clearRoute = () => {
    clear();
    wsClient.send({ op: 'clear_route' });
  };

  return (
    <div className="panel">
      <h3 style={{ marginTop: 0 }}>Routing</h3>
      <p className="hint">
        Map RMB adds waypoints · send as route ({route?.state ?? '—'})
      </p>
      <div className="row" style={{ gap: 8, marginBottom: 8, flexWrap: 'wrap' }}>
        <button type="button" disabled={!connected || !waypoints.length} onClick={sendRoute}>
          set_route ({waypoints.length})
        </button>
        <button type="button" disabled={!connected} onClick={clearRoute}>
          clear_route
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
