import { useWaypointStore, waypointColor } from '@/store/waypointStore';
import { goNavigation, stopNavigation } from '@/store/navActions';
import { useDataStore } from '@/store/dataStore';

/** Compact waypoint list — Map canvas is the primary editor. */
export function WaypointsPanel() {
  const connected = useDataStore((s) => s.connected);
  const { waypoints, selectedId, remove, moveUp, moveDown, select, update } =
    useWaypointStore();

  const selected = waypoints.find((w) => w.id === selectedId) ?? null;
  const n = waypoints.length;

  return (
    <div className="panel">
      <h3 style={{ marginTop: 0 }}>航点</h3>
      <p className="hint">在 Map 选「导航」拖放加点 · 此处可排序 / 微调</p>

      <div className="row" style={{ gap: 8, marginBottom: 10, flexWrap: 'wrap' }}>
        <button
          type="button"
          className="btn-primary"
          disabled={!connected || !n}
          onClick={() => goNavigation()}
        >
          {n <= 1 ? '出发' : `出发 (${n})`}
        </button>
        <button type="button" disabled={!n} onClick={() => stopNavigation(true)}>
          停止清空
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
                <span className="wp-swatch" style={{ background: color }} />
                <span>
                  {idx + 1}. ({w.x.toFixed(2)}, {w.y.toFixed(2)}){' '}
                  {(((w.yaw ?? 0) * 180) / Math.PI).toFixed(0)}°
                </span>
              </span>
              <span className="wp-actions">
                <button
                  type="button"
                  title="上移"
                  onClick={(e) => {
                    e.stopPropagation();
                    moveUp(w.id);
                  }}
                >
                  ↑
                </button>
                <button
                  type="button"
                  title="下移"
                  onClick={(e) => {
                    e.stopPropagation();
                    moveDown(w.id);
                  }}
                >
                  ↓
                </button>
                <button
                  type="button"
                  title="删除"
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
          <div className="wp-edit-title">微调</div>
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
            <span>朝向°</span>
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

      {!n ? <p className="muted">暂无航点</p> : null}
    </div>
  );
}
