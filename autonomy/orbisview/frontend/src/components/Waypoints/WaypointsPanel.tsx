import { useWaypointStore } from '@/store/waypointStore';
import { wsClient } from '@/store/websocket/client';
import { useDataStore } from '@/store/dataStore';

export function WaypointsPanel() {
  const connected = useDataStore((s) => s.connected);
  const { waypoints, selectedId, remove, moveUp, moveDown, select, clear, add } =
    useWaypointStore();

  const sendSelected = () => {
    const wp = waypoints.find((w) => w.id === selectedId);
    if (!wp || !connected) return;
    wsClient.send({ op: 'set_goal', x: wp.x, y: wp.y });
  };

  return (
    <div className="panel">
      <h3 style={{ marginTop: 0 }}>Waypoints</h3>
      <p className="hint">Add from Map (right-click) or manually below.</p>
      <div className="row" style={{ gap: 8, marginBottom: 8, flexWrap: 'wrap' }}>
        <button
          type="button"
          disabled={!connected}
          onClick={() => add(0, 0, 'origin')}
        >
          Add (0,0)
        </button>
        <button type="button" onClick={sendSelected} disabled={!connected || !selectedId}>
          set_goal selected
        </button>
        <button type="button" onClick={clear} disabled={!waypoints.length}>
          Clear all
        </button>
      </div>
      <ul className="wp-list">
        {waypoints.map((w, idx) => (
          <li
            key={w.id}
            className={w.id === selectedId ? 'wp-item active' : 'wp-item'}
            onClick={() => select(w.id)}
          >
            <span>
              #{idx + 1} ({w.x.toFixed(2)}, {w.y.toFixed(2)})
              {w.label ? ` · ${w.label}` : ''}
            </span>
            <span className="wp-actions">
              <button type="button" onClick={(e) => { e.stopPropagation(); moveUp(w.id); }}>
                ↑
              </button>
              <button type="button" onClick={(e) => { e.stopPropagation(); moveDown(w.id); }}>
                ↓
              </button>
              <button type="button" onClick={(e) => { e.stopPropagation(); remove(w.id); }}>
                ×
              </button>
            </span>
          </li>
        ))}
      </ul>
      {!waypoints.length ? <p className="muted">empty</p> : null}
    </div>
  );
}
