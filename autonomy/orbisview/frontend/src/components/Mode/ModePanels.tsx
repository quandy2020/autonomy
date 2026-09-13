import { useEffect, useMemo, useState } from 'react';
import { useDataStore } from '@/store/dataStore';
import { wsClient } from '@/store/websocket/client';
import { HmiPanel, ComponentsPanel } from '@/components/Hmi/HmiPanels';

/** Dreamview+ Mode Settings–inspired: mode, modules, operations (no cyber_launch). */
export function ModeSettingsPanel() {
  const connected = useDataStore((s) => s.connected);

  return (
    <div className="panel mode-settings">
      <h3 style={{ marginTop: 0 }}>Mode Settings</h3>
      <HmiPanel />
      <hr className="sep" />
      <h4>Modules</h4>
      <ComponentsPanel />
      <hr className="sep" />
      <h4>Operations</h4>
      <div className="row" style={{ gap: 8, flexWrap: 'wrap' }}>
        <button
          type="button"
          disabled={!connected}
          onClick={() =>
            wsClient.send({
              op: 'record_start',
              path: '/tmp/orbisview_record.jsonl',
            })
          }
        >
          Record
        </button>
        <button
          type="button"
          disabled={!connected}
          onClick={() => wsClient.send({ op: 'record_stop' })}
        >
          Stop Record
        </button>
        <button
          type="button"
          disabled={!connected}
          onClick={() =>
            wsClient.send({
              op: 'playback_start',
              path: '/tmp/orbisview_record.jsonl',
              speed: 1,
              loop: false,
            })
          }
        >
          Play Bag
        </button>
        <button
          type="button"
          disabled={!connected}
          onClick={() => wsClient.send({ op: 'playback_stop' })}
        >
          Stop Play
        </button>
      </div>
      <p className="hint">Aligns with DV+ Mode/Modules/Operations; soft HMI only.</p>
    </div>
  );
}

export function ModuleDelayPanel() {
  const envelopes = useDataStore((s) => s.envelopes);
  const connected = useDataStore((s) => s.connected);

  const rows = useMemo(() => {
    return Object.values(envelopes)
      .map((e) => {
        const ageMs =
          e.timestamp > 1e12
            ? Date.now() - e.timestamp / 1e6
            : Date.now() - e.timestamp;
        return {
          channel: e.channel,
          schema: e.schema,
          delay_ms: Math.max(0, ageMs),
          stale: !!e.stale,
        };
      })
      .sort((a, b) => b.delay_ms - a.delay_ms)
      .slice(0, 24);
  }, [envelopes]);

  return (
    <div className="panel">
      <h3 style={{ marginTop: 0 }}>Module Delay</h3>
      {!connected ? <p className="muted">offline</p> : null}
      <table className="stats-table">
        <thead>
          <tr>
            <th>channel</th>
            <th>delay ms</th>
            <th>stale</th>
          </tr>
        </thead>
        <tbody>
          {rows.map((r) => (
            <tr key={r.channel}>
              <td title={r.schema}>{r.channel.replace('/orbisview/mock/', '')}</td>
              <td className={r.delay_ms > 500 ? 'warn' : 'ok'}>{r.delay_ms.toFixed(0)}</td>
              <td>{r.stale ? 'yes' : ''}</td>
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  );
}

export function ResourceManagerPanel() {
  const connected = useDataStore((s) => s.connected);
  const pushLog = useDataStore((s) => s.pushLog);
  const [bags, setBags] = useState<{ name: string; path: string }[]>([]);
  const [dir, setDir] = useState('/tmp');

  useEffect(() => {
    const off = wsClient.onMessage((msg) => {
      if (msg.op === 'local_bags') {
        setBags(msg.bags ?? []);
        pushLog(`local_bags=${(msg.bags ?? []).length}`);
      }
    });
    return off;
  }, [pushLog]);

  return (
    <div className="panel">
      <h3 style={{ marginTop: 0 }}>Resource Manager</h3>
      <p className="hint">Local bags only (no Apollo cloud / V2X).</p>
      <div className="row" style={{ gap: 8, marginBottom: 8 }}>
        <input value={dir} onChange={(e) => setDir(e.target.value)} size={18} />
        <button
          type="button"
          disabled={!connected}
          onClick={() => wsClient.send({ op: 'list_local_bags', path: dir })}
        >
          Scan
        </button>
      </div>
      <ul className="wp-list">
        {bags.map((b) => (
          <li key={b.path} className="wp-item">
            <span>{b.name}</span>
            <button
              type="button"
              disabled={!connected}
              onClick={() =>
                wsClient.send({
                  op: 'playback_start',
                  path: b.path,
                  speed: 1,
                  loop: false,
                })
              }
            >
              Play
            </button>
          </li>
        ))}
      </ul>
      {!bags.length ? <p className="muted">empty — scan /tmp for *.jsonl</p> : null}
    </div>
  );
}
