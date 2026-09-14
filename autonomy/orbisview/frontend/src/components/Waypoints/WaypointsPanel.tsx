import { useWaypointStore, waypointColor } from '@/store/waypointStore';
import { wsClient } from '@/store/websocket/client';
import { useDataStore } from '@/store/dataStore';

export function WaypointsPanel() {
  const connected = useDataStore((s) => s.connected);
  const { waypoints, selectedId, remove, moveUp, moveDown, select, clear, add, update } =
    useWaypointStore();

  const selected = waypoints.find((w) => w.id === selectedId) ?? null;

  const sendSelected = () => {
    if (!selected || !connected) return;
    wsClient.send({ op: 'set_goal', x: selected.x, y: selected.y, yaw: selected.yaw ?? 0 });
  };

  return (
    <div className="panel">
      <h3 style={{ marginTop: 0 }}>Waypoints</h3>
      <p className="hint">Map 多点工具添加 · 列表 / 2D / 3D 可选中拖拽修改</p>
      <div className="row" style={{ gap: 8, marginBottom: 8, flexWrap: 'wrap' }}>
        <button type="button" disabled={!connected} onClick={() => add(0, 0, 0, 'origin')}>
          Add (0,0)
        </button>
        <button type="button" onClick={sendSelected} disabled={!connected || !selected}>
          set_goal selected
        </button>
        <button type="button" onClick={clear} disabled={!waypoints.length}>
          Clear all
        </button>
      </div>
      <ul className="wp-list">
        {waypoints.map((w, idx) => {
          const color = waypointColor(idx);
          const active = w.id === selectedId;
          return (
            <li
              key={w.id}
              className={active ? 'wp-item active' : 'wp-item'}
              style={{
                borderColor: active ? color : undefined,
                boxShadow: active ? `inset 3px 0 0 ${color}` : undefined,
              }}
              onClick={() => select(w.id)}
            >
              <span className="wp-item-main">
                <span className="wp-swatch" style={{ background: color }} title={color} />
                <span>
                  #{idx + 1} ({w.x.toFixed(2)}, {w.y.toFixed(2)},{' '}
                  {(((w.yaw ?? 0) * 180) / Math.PI).toFixed(0)}°)
                  {w.label ? ` · ${w.label}` : ''}
                </span>
              </span>
              <span className="wp-actions">
                <button
                  type="button"
                  onClick={(e) => {
                    e.stopPropagation();
                    moveUp(w.id);
                  }}
                >
                  ↑
                </button>
                <button
                  type="button"
                  onClick={(e) => {
                    e.stopPropagation();
                    moveDown(w.id);
                  }}
                >
                  ↓
                </button>
                <button
                  type="button"
                  onClick={(e) => {
                    e.stopPropagation();
                    remove(w.id);
                  }}
                >
                  ×
                </button>
              </span>
            </li>
          );
        })}
      </ul>
      {selected ? (
        <div className="wp-edit">
          <div className="wp-edit-title">编辑选中点</div>
          <label className="wp-edit-field">
            <span>x</span>
            <input
              type="number"
              step={0.01}
              value={Number(selected.x.toFixed(3))}
              onChange={(e) => update(selected.id, { x: Number(e.target.value) })}
            />
          </label>
          <label className="wp-edit-field">
            <span>y</span>
            <input
              type="number"
              step={0.01}
              value={Number(selected.y.toFixed(3))}
              onChange={(e) => update(selected.id, { y: Number(e.target.value) })}
            />
          </label>
          <label className="wp-edit-field">
            <span>yaw°</span>
            <input
              type="number"
              step={1}
              value={Number((((selected.yaw ?? 0) * 180) / Math.PI).toFixed(1))}
              onChange={(e) =>
                update(selected.id, { yaw: (Number(e.target.value) * Math.PI) / 180 })
              }
            />
          </label>
        </div>
      ) : null}
      {!waypoints.length ? <p className="muted">empty</p> : null}
    </div>
  );
}
