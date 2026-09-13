import { useEffect, useState } from 'react';
import { wsClient } from '@/store/websocket/client';
import { useDataStore } from '@/store/dataStore';

export function DiagnosticsPanel() {
  const connected = useDataStore((s) => s.connected);
  const log = useDataStore((s) => s.log);
  const [status, setStatus] = useState('');
  const [plugins, setPlugins] = useState('');

  useEffect(() => {
    if (!connected) return;
    const off = wsClient.onMessage((msg) => {
      if (msg.op === 'status') setStatus(JSON.stringify(msg, null, 2));
      if ((msg as { op?: string }).op === 'plugins') {
        setPlugins(JSON.stringify(msg, null, 2));
      }
    });
    wsClient.status();
    wsClient.send({ op: 'list_plugins' });
    return off;
  }, [connected]);

  return (
    <div className="panel log-panel">
      <h3>Diagnostics</h3>
      <button type="button" onClick={() => wsClient.status()}>Refresh status</button>
      <pre>{status || '—'}</pre>
      <h3>Plugins</h3>
      <pre>{plugins || '—'}</pre>
      <h3>Recent log</h3>
      <pre>{log.slice(0, 30).join('\n')}</pre>
    </div>
  );
}
