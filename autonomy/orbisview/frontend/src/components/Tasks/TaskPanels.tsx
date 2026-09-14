import { useEffect, useMemo, useState } from 'react';
import { useDataStore } from '@/store/dataStore';
import { SCHEMAS } from '@/store/websocket/types';
import { wsClient } from '@/store/websocket/client';

function TaskPanel({ schema, title }: { schema: string; title: string }) {
  const envelopes = useDataStore((s) => s.envelopes);
  const payload = useMemo(() => {
    const e = Object.values(envelopes).find((x) => x.schema === schema);
    return e?.payload ?? null;
  }, [envelopes, schema]);

  return (
    <div className="panel">
      <h3>{title}</h3>
      <pre>{payload ? JSON.stringify(payload, null, 2) : 'Waiting for data…'}</pre>
    </div>
  );
}

export function ExplorationPanel() {
  return <TaskPanel schema={SCHEMAS.Exploration} title="Exploration" />;
}

export function NavigationPanel() {
  const connected = useDataStore((s) => s.connected);
  const envelopes = useDataStore((s) => s.envelopes);
  const [ack, setAck] = useState('');
  const payload = useMemo(() => {
    const e = Object.values(envelopes).find((x) => x.schema === SCHEMAS.Navigation);
    return e?.payload ?? null;
  }, [envelopes]);

  useEffect(() => {
    if (!connected) return;
    return wsClient.onMessage((msg) => {
      const op = (msg as { op?: string }).op;
      if (op === 'goal_set' || op === 'goal_cleared') {
        setAck(JSON.stringify(msg));
      }
    });
  }, [connected]);

  return (
    <div className="panel">
      <h3>Navigation</h3>
      <p className="hint">Set goal by clicking Map 2D (sends set_goal).</p>
      <div className="row" style={{ gap: 8 }}>
        <button
          type="button"
          disabled={!connected}
          onClick={() => wsClient.send({ op: 'set_goal', x: 2, y: 1 })}
        >
          Goal (2,1)
        </button>
        <button
          type="button"
          disabled={!connected}
          onClick={() => wsClient.send({ op: 'clear_goal' })}
        >
          Clear
        </button>
      </div>
      {ack ? <pre style={{ marginTop: 8 }}>{ack}</pre> : null}
      <pre style={{ marginTop: 8 }}>
        {payload ? JSON.stringify(payload, null, 2) : 'Waiting for data…'}
      </pre>
    </div>
  );
}

export { MappingPanel } from './MappingPanel';
