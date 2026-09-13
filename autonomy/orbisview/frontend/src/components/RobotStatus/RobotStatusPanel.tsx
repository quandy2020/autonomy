import { useMemo } from 'react';
import { useDataStore } from '@/store/dataStore';
import { SCHEMAS } from '@/store/websocket/types';

interface Pose2D {
  x: number;
  y: number;
  yaw?: number;
}
interface Twist2D {
  vx: number;
  wz: number;
}
interface NavPayload {
  state?: string;
  goal?: Pose2D;
  has_goal?: boolean;
  distance_remaining?: number;
}

function asPayload<T>(env: { payload?: unknown } | undefined): T | null {
  if (!env?.payload || typeof env.payload !== 'object') return null;
  return env.payload as T;
}

export function RobotStatusPanel() {
  const envelopes = useDataStore((s) => s.envelopes);
  const connected = useDataStore((s) => s.connected);
  const connectionState = useDataStore((s) => s.connectionState);

  const pose = useMemo(() => {
    const e = Object.values(envelopes).find((x) => x.schema === SCHEMAS.Pose2D);
    return asPayload<Pose2D>(e);
  }, [envelopes]);

  const twist = useMemo(() => {
    const e = Object.values(envelopes).find((x) => x.schema === SCHEMAS.Twist2D);
    return asPayload<Twist2D>(e);
  }, [envelopes]);

  const nav = useMemo(() => {
    const e = Object.values(envelopes).find((x) => x.schema === SCHEMAS.Navigation);
    return asPayload<NavPayload>(e);
  }, [envelopes]);

  return (
    <div className="panel">
      <h3 style={{ marginTop: 0 }}>Robot Status</h3>
      <table className="stats-table">
        <tbody>
          <tr>
            <th>Connection</th>
            <td className={connected ? 'ok' : 'bad'}>{connectionState}</td>
          </tr>
          <tr>
            <th>Pose</th>
            <td>
              {pose
                ? `x=${pose.x.toFixed(2)} y=${pose.y.toFixed(2)} yaw=${(pose.yaw ?? 0).toFixed(2)}`
                : '—'}
            </td>
          </tr>
          <tr>
            <th>Twist</th>
            <td>
              {twist
                ? `vx=${twist.vx.toFixed(2)} wz=${twist.wz.toFixed(2)}`
                : '—'}
            </td>
          </tr>
          <tr>
            <th>Nav</th>
            <td>{nav?.state ?? '—'}</td>
          </tr>
          <tr>
            <th>Goal</th>
            <td>
              {nav?.has_goal && nav.goal
                ? `(${nav.goal.x.toFixed(2)}, ${nav.goal.y.toFixed(2)}) rem=${(
                    nav.distance_remaining ?? 0
                  ).toFixed(2)}`
                : 'none'}
            </td>
          </tr>
        </tbody>
      </table>
    </div>
  );
}
