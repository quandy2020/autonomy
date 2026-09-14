import { useEffect, useMemo, useRef, useState } from 'react';
import { useDataStore } from '@/store/dataStore';
import { SCHEMAS } from '@/store/websocket/types';

function asObj(env: { payload?: unknown } | undefined): Record<string, unknown> | null {
  if (!env?.payload || typeof env.payload !== 'object') return null;
  return env.payload as Record<string, unknown>;
}

function num(v: unknown): number | null {
  return typeof v === 'number' && Number.isFinite(v) ? v : null;
}

/** Semi-circular arc gauge (SVG). value in [0,1]. */
function ArcGauge({
  value01,
  color,
  track = 'rgba(255,255,255,0.12)',
  size = 72,
  stroke = 7,
}: {
  value01: number;
  color: string;
  track?: string;
  size?: number;
  stroke?: number;
}) {
  const r = (size - stroke) / 2;
  const c = 2 * Math.PI * r;
  const half = c / 2;
  const clamped = Math.max(0, Math.min(1, value01));
  const dash = half * clamped;
  return (
    <svg width={size} height={size * 0.62} viewBox={`0 0 ${size} ${size * 0.62}`} aria-hidden>
      <g transform={`translate(${size / 2}, ${size / 2})`}>
        <circle
          r={r}
          fill="none"
          stroke={track}
          strokeWidth={stroke}
          strokeLinecap="round"
          strokeDasharray={`${half} ${c}`}
          transform="rotate(180)"
        />
        <circle
          r={r}
          fill="none"
          stroke={color}
          strokeWidth={stroke}
          strokeLinecap="round"
          strokeDasharray={`${dash} ${c}`}
          transform="rotate(180)"
          style={{ filter: `drop-shadow(0 0 6px ${color}88)` }}
        />
      </g>
    </svg>
  );
}

function BatteryBars({ pct }: { pct: number | null }) {
  const level = pct == null ? -1 : Math.round(Math.max(0, Math.min(100, pct)) / 20);
  return (
    <div className="map-gauge-batt" aria-hidden>
      {[0, 1, 2, 3, 4].map((i) => (
        <span
          key={i}
          className={`map-gauge-batt-seg${level < 0 ? '' : i < level ? ' on' : ''}${
            pct != null && pct <= 20 && i < level ? ' low' : ''
          }`}
        />
      ))}
    </div>
  );
}

/**
 * Transparent instrument cluster for Map (top-left): speed / odometer / battery.
 */
export function MapInstrumentCluster() {
  const envelopes = useDataStore((s) => s.envelopes);
  const lastPose = useRef<{ x: number; y: number } | null>(null);
  const [tripM, setTripM] = useState(0);

  const twist = useMemo(() => {
    const e = Object.values(envelopes).find((x) => x.schema === SCHEMAS.Twist2D);
    return asObj(e);
  }, [envelopes]);

  const chassis = useMemo(() => {
    const e = Object.values(envelopes).find((x) => x.schema === SCHEMAS.ChassisState);
    return asObj(e);
  }, [envelopes]);

  const pose = useMemo(() => {
    const e = Object.values(envelopes).find((x) => x.schema === SCHEMAS.Pose2D);
    return asObj(e);
  }, [envelopes]);

  const batteryPct = useMemo(() => {
    for (const e of Object.values(envelopes)) {
      const hay = `${e.channel} ${e.schema}`.toLowerCase();
      if (!/battery|robotstate|robot_state/.test(hay)) continue;
      const p = asObj(e);
      if (!p) continue;
      const pct = num(p.percentage) ?? num(p.battery_percent) ?? num(p.batteryPercent);
      if (pct != null) return pct <= 1.0001 ? pct * 100 : pct;
    }
    return null;
  }, [envelopes]);

  const charging = useMemo(() => {
    for (const e of Object.values(envelopes)) {
      const p = asObj(e);
      if (!p) continue;
      if (p.is_charging === true) return true;
      if (typeof p.power_supply_status === 'number' && p.power_supply_status === 1) return true;
    }
    return false;
  }, [envelopes]);

  useEffect(() => {
    const x = num(pose?.x);
    const y = num(pose?.y);
    if (x == null || y == null) return;
    const prev = lastPose.current;
    if (prev) {
      const d = Math.hypot(x - prev.x, y - prev.y);
      if (d > 1e-4 && d < 5) setTripM((m) => m + d);
    }
    lastPose.current = { x, y };
  }, [pose]);

  const vx = num(twist?.vx) ?? num(chassis?.vx) ?? 0;
  const wz = num(twist?.wz) ?? num(chassis?.wz) ?? 0;
  const speedMs = Math.abs(vx);
  const speed01 = Math.min(1, speedMs / 2.0);
  const yaw01 = Math.min(1, Math.abs(wz) / 1.5);
  const odomKm = tripM / 1000;
  const batt = batteryPct;
  const batt01 = batt == null ? 0 : Math.max(0, Math.min(1, batt / 100));

  const speedColor =
    speed01 > 0.85 ? '#ef9a9a' : speed01 > 0.55 ? '#ffcc80' : '#69f0ae';
  const yawColor =
    yaw01 > 0.85 ? '#ef9a9a' : yaw01 > 0.55 ? '#ffcc80' : '#4fc3f7';
  const battColor =
    batt == null ? '#6b7f93' : batt <= 20 ? '#ef9a9a' : batt <= 40 ? '#ffcc80' : '#4fc3f7';

  return (
    <aside className="map-gauges" aria-label="Vehicle instruments">
      <div className="map-gauge-card map-gauge-card-speed">
        <div className="map-gauge-head">
          <span className="map-gauge-title">Speed</span>
          <span className="map-gauge-unit">v · ω</span>
        </div>
        <div className="map-gauge-speed-grid">
          <div className="map-gauge-speed-cell">
            <ArcGauge value01={speed01} color={speedColor} size={56} stroke={6} />
            <div className="map-gauge-metric">
              <span className="map-gauge-metric-label">v</span>
              <strong>{vx.toFixed(2)}</strong>
              <em>m/s</em>
            </div>
          </div>
          <div className="map-gauge-speed-cell">
            <ArcGauge value01={yaw01} color={yawColor} size={56} stroke={6} />
            <div className="map-gauge-metric">
              <span className="map-gauge-metric-label">ω</span>
              <strong>{wz.toFixed(2)}</strong>
              <em>rad/s</em>
            </div>
          </div>
        </div>
      </div>
      <div className="map-gauge-card">
        <div className="map-gauge-head">
          <span className="map-gauge-title">Odometer</span>
          <span className="map-gauge-unit">trip</span>
        </div>
        <div className="map-gauge-body map-gauge-body-flat">
          <div className="map-gauge-odo">
            <strong>{odomKm < 1 ? tripM.toFixed(1) : odomKm.toFixed(2)}</strong>
            <em>{odomKm < 1 ? 'm' : 'km'}</em>
          </div>
          <div className="map-gauge-odo-bar">
            <i style={{ width: `${Math.min(100, tripM % 100)}%` }} />
          </div>
        </div>
      </div>

      <div className="map-gauge-card">
        <div className="map-gauge-head">
          <span className="map-gauge-title">Battery</span>
          <span className="map-gauge-unit">{charging ? 'CHG' : '%'}</span>
        </div>
        <div className="map-gauge-body">
          <ArcGauge value01={batt == null ? 0 : batt01} color={battColor} />
          <div className="map-gauge-readout">
            <strong>{batt == null ? '—' : batt.toFixed(0)}</strong>
            <em>{batt == null ? 'no data' : charging ? 'charging' : 'SoC'}</em>
            <BatteryBars pct={batt} />
          </div>
        </div>
      </div>
    </aside>
  );
}
