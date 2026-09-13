import { useMemo } from 'react';
import { useDataStore } from '@/store/dataStore';
import { SCHEMAS } from '@/store/websocket/types';

interface Chassis {
  vx?: number;
  wz?: number;
  gear?: string;
  throttle?: number;
  brake?: number;
  steering?: number;
  driving_mode?: string;
}

function asPayload<T>(env: { payload?: unknown } | undefined): T | null {
  if (!env?.payload || typeof env.payload !== 'object') return null;
  return env.payload as T;
}

export function DashboardPanel() {
  const envelopes = useDataStore((s) => s.envelopes);
  const chassis = useMemo(() => {
    const e = Object.values(envelopes).find((x) => x.schema === SCHEMAS.ChassisState);
    return asPayload<Chassis>(e);
  }, [envelopes]);

  const vx = chassis?.vx ?? 0;
  const wz = chassis?.wz ?? 0;
  const throttle = chassis?.throttle ?? 0;
  const brake = chassis?.brake ?? 0;
  const steering = chassis?.steering ?? 0;

  return (
    <div className="panel dashboard">
      <h3 style={{ marginTop: 0 }}>Dashboard</h3>
      <div className="dash-grid">
        <div className="dash-card">
          <div className="dash-label">Speed</div>
          <div className="dash-value">{vx.toFixed(2)} m/s</div>
        </div>
        <div className="dash-card">
          <div className="dash-label">Yaw rate</div>
          <div className="dash-value">{wz.toFixed(2)} rad/s</div>
        </div>
        <div className="dash-card">
          <div className="dash-label">Gear</div>
          <div className="dash-value">{chassis?.gear ?? '—'}</div>
        </div>
        <div className="dash-card">
          <div className="dash-label">Mode</div>
          <div className="dash-value">{chassis?.driving_mode ?? '—'}</div>
        </div>
      </div>
      <div className="dash-bars">
        <label>
          Throttle
          <meter min={0} max={1} value={throttle} />
          <span className="muted">{throttle.toFixed(2)}</span>
        </label>
        <label>
          Brake
          <meter min={0} max={1} value={brake} />
          <span className="muted">{brake.toFixed(2)}</span>
        </label>
        <label>
          Steering
          <input type="range" min={-1} max={1} step={0.01} value={steering} readOnly />
          <span className="muted">{steering.toFixed(2)}</span>
        </label>
      </div>
    </div>
  );
}
