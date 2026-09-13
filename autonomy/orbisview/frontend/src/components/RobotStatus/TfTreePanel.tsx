import { useMemo } from 'react';
import { useDataStore } from '@/store/dataStore';
import { SCHEMAS } from '@/store/websocket/types';

interface TfXform {
  parent: string;
  child: string;
  x: number;
  y: number;
  yaw?: number;
}
interface TfTree {
  transforms: TfXform[];
}

function asPayload<T>(env: { payload?: unknown } | undefined): T | null {
  if (!env?.payload || typeof env.payload !== 'object') return null;
  return env.payload as T;
}

export function TfTreePanel() {
  const envelopes = useDataStore((s) => s.envelopes);
  const tf = useMemo(() => {
    const e = Object.values(envelopes).find((x) => x.schema === SCHEMAS.TfTree);
    return asPayload<TfTree>(e);
  }, [envelopes]);

  const list = tf?.transforms ?? [];

  return (
    <div className="panel">
      <h3 style={{ marginTop: 0 }}>TF Tree</h3>
      {list.length === 0 ? (
        <p className="muted">no transforms</p>
      ) : (
        <ul className="tf-list">
          {list.map((t, i) => (
            <li key={`${t.parent}-${t.child}-${i}`}>
              <code>
                {t.parent} → {t.child}
              </code>
              <span className="muted">
                {' '}
                ({t.x.toFixed(2)}, {t.y.toFixed(2)}, yaw=
                {(t.yaw ?? 0).toFixed(2)})
              </span>
            </li>
          ))}
        </ul>
      )}
      <div className="tf-preview">
        {list.map((t, i) => (
          <div key={i} className="tf-edge">
            <span className="tf-node">{t.parent}</span>
            <span className="tf-arrow">→</span>
            <span className="tf-node">{t.child}</span>
          </div>
        ))}
      </div>
    </div>
  );
}
