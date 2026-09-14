import { useEffect, useState } from 'react';
import { useDataStore } from '@/store/dataStore';
import { wsClient } from '@/store/websocket/client';

export function ChannelLogPanel() {
  const log = useDataStore((s) => s.log);
  const channels = useDataStore((s) => s.channels);
  const envelopes = useDataStore((s) => s.envelopes);
  const subscribed = useDataStore((s) => s.subscribed);
  const markSubscribed = useDataStore((s) => s.markSubscribed);
  const markUnsubscribed = useDataStore((s) => s.markUnsubscribed);
  const connected = useDataStore((s) => s.connected);
  const [hzByChannel, setHzByChannel] = useState<Record<string, number>>({});

  useEffect(() => {
    if (!connected) {
      setHzByChannel({});
      return;
    }
    const off = wsClient.onMessage((msg) => {
      if (msg.op !== 'channel_stats') return;
      const next: Record<string, number> = {};
      for (const row of msg.channels ?? []) {
        next[row.name] = row.hz;
      }
      setHzByChannel(next);
    });
    wsClient.send({ op: 'channel_stats' });
    const timer = window.setInterval(() => wsClient.send({ op: 'channel_stats' }), 1000);
    return () => {
      off();
      window.clearInterval(timer);
    };
  }, [connected]);

  const toggle = (name: string) => {
    if (!connected) return;
    if (subscribed[name] != null) {
      wsClient.unsubscribe(name);
      markUnsubscribed(name);
    } else {
      wsClient.subscribe(name, 0);
      markSubscribed(name, 0);
    }
  };

  const subscribeAllVisible = () => {
    if (!connected) return;
    channels.forEach((c) => {
      if (subscribed[c.name] != null) return;
      wsClient.subscribe(c.name, 0);
      markSubscribed(c.name, 0);
    });
  };

  return (
    <div className="panel log-panel">
      <h3>Channels ({channels.length})</h3>
      <div className="row" style={{ gap: 8, marginBottom: 8 }}>
        <button type="button" onClick={() => wsClient.listChannels()} disabled={!connected}>
          Refresh list
        </button>
        <button type="button" onClick={subscribeAllVisible} disabled={!connected}>
          Subscribe all
        </button>
      </div>
      <ul>
        {channels.map((c) => {
          const env = envelopes[c.name];
          const isSub = subscribed[c.name] != null;
          const maxHz = subscribed[c.name];
          const hz = hzByChannel[c.name];
          return (
            <li key={c.name} className="channel-row">
              <label className="row channel-row-main">
                <input
                  type="checkbox"
                  checked={isSub}
                  disabled={!connected}
                  onChange={() => toggle(c.name)}
                />
                <span className="channel-meta">
                  <span className="channel-name">{c.name}</span>
                  <span className="channel-subrow">
                    <span className="muted channel-schema" title={c.schema}>
                      {c.schema}
                    </span>
                    {c.mock ? <span className="muted">mock</span> : null}
                    <span
                      className={`hz-badge ${isSub ? 'hz-on' : 'hz-off'}`}
                      title={
                        isSub && maxHz != null
                          ? `measured · subscribe max ${maxHz} Hz`
                          : 'measured Hz'
                      }
                    >
                      {hz != null && Number.isFinite(hz) ? `${hz.toFixed(1)} Hz` : '— Hz'}
                    </span>
                    {isSub && maxHz != null ? (
                      <span className="muted channel-maxhz">max {maxHz}</span>
                    ) : null}
                    {env?.stale ? <span className="stale-badge">stale</span> : null}
                    {env?.unsupported ? (
                      <span className="unsupported-badge">unsupported</span>
                    ) : null}
                  </span>
                </span>
              </label>
            </li>
          );
        })}
      </ul>
      <h3>Envelope log</h3>
      <pre>{log.join('\n')}</pre>
    </div>
  );
}
