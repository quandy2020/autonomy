import { useEffect, useState } from 'react';
import { wsClient } from '@/store/websocket/client';
import { useDataStore } from '@/store/dataStore';

interface StatRow {
  name: string;
  count: number;
  hz: number;
  latency_ms: number;
}

export function StatsPanel() {
  const [rows, setRows] = useState<StatRow[]>([]);
  const connected = useDataStore((s) => s.connected);

  useEffect(() => {
    if (!connected) return;
    const off = wsClient.onMessage((msg) => {
      if (msg.op === 'channel_stats') {
        setRows(msg.channels ?? []);
      }
    });
    const t = window.setInterval(() => wsClient.send({ op: 'channel_stats' }), 1000);
    return () => {
      off();
      window.clearInterval(t);
    };
  }, [connected]);

  return (
    <div className="panel">
      <h3>Channel Hz / Latency</h3>
      <table className="stats-table">
        <thead>
          <tr>
            <th>channel</th>
            <th>count</th>
            <th>Hz</th>
            <th>latency ms</th>
          </tr>
        </thead>
        <tbody>
          {rows.map((r) => (
            <tr key={r.name}>
              <td>{r.name}</td>
              <td>{r.count}</td>
              <td>{r.hz.toFixed(1)}</td>
              <td>{r.latency_ms.toFixed(1)}</td>
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  );
}
