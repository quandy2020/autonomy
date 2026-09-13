import { useMemo } from 'react';
import { useDataStore } from '@/store/dataStore';
import { SCHEMAS } from '@/store/websocket/types';
import { wsClient } from '@/store/websocket/client';

interface ComponentRow {
  id: string;
  title?: string;
  expected?: boolean;
  healthy?: boolean;
  delay_ms?: number;
  status?: string;
}

interface HmiStatus {
  mode?: string;
  modes?: { id: string; title: string }[];
}

function asPayload<T>(env: { payload?: unknown } | undefined): T | null {
  if (!env?.payload || typeof env.payload !== 'object') return null;
  return env.payload as T;
}

export function ComponentsPanel() {
  const envelopes = useDataStore((s) => s.envelopes);
  const connected = useDataStore((s) => s.connected);

  const comps = useMemo(() => {
    const e = Object.values(envelopes).find((x) => x.schema === SCHEMAS.ComponentsStatus);
    return asPayload<{ components?: ComponentRow[] }>(e)?.components ?? [];
  }, [envelopes]);

  return (
    <div className="panel">
      <h3 style={{ marginTop: 0 }}>Components</h3>
      <table className="stats-table">
        <thead>
          <tr>
            <th>Module</th>
            <th>Status</th>
            <th>Delay</th>
            <th />
          </tr>
        </thead>
        <tbody>
          {comps.map((c) => (
            <tr key={c.id}>
              <td>{c.title ?? c.id}</td>
              <td className={c.status === 'OK' ? 'ok' : c.status === 'DISABLED' ? 'muted' : 'bad'}>
                {c.status ?? '—'}
              </td>
              <td>{c.delay_ms != null ? `${c.delay_ms.toFixed(0)} ms` : '—'}</td>
              <td>
                <button
                  type="button"
                  disabled={!connected}
                  onClick={() =>
                    wsClient.send({
                      op: 'hmi_module_action',
                      id: c.id,
                      action: c.expected ? 'stop' : 'start',
                    })
                  }
                >
                  {c.expected ? 'Disable' : 'Enable'}
                </button>
              </td>
            </tr>
          ))}
        </tbody>
      </table>
      {!comps.length ? <p className="muted">no components</p> : null}
    </div>
  );
}

export function HmiPanel() {
  const envelopes = useDataStore((s) => s.envelopes);
  const connected = useDataStore((s) => s.connected);
  const hmi = useMemo(() => {
    const e = Object.values(envelopes).find((x) => x.schema === SCHEMAS.HmiStatus);
    return asPayload<HmiStatus>(e);
  }, [envelopes]);

  const modes = hmi?.modes ?? [
    { id: 'default', title: 'Default' },
    { id: 'pnc', title: 'PNC' },
    { id: 'mapping', title: 'Mapping' },
  ];

  return (
    <div className="panel">
      <h3 style={{ marginTop: 0 }}>HMI Mode</h3>
      <p className="muted">current: {hmi?.mode ?? '—'}</p>
      <div className="row" style={{ gap: 8, flexWrap: 'wrap' }}>
        {modes.map((m) => (
          <button
            key={m.id}
            type="button"
            className={hmi?.mode === m.id ? 'tab active' : 'tab'}
            disabled={!connected}
            onClick={() => wsClient.send({ op: 'hmi_set_mode', mode: m.id })}
          >
            {m.title}
          </button>
        ))}
        <button type="button" disabled={!connected} onClick={() => wsClient.send({ op: 'hmi_status' })}>
          Refresh
        </button>
      </div>
      <p className="hint">Soft mode switch — no process spawn (OrbisView HMI).</p>
    </div>
  );
}
