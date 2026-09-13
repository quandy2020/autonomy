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

  const toggle = (name: string) => {
    if (!connected) return;
    if (subscribed[name] != null) {
      wsClient.unsubscribe(name);
      markUnsubscribed(name);
    } else {
      wsClient.subscribe(name, 20);
      markSubscribed(name, 20);
    }
  };

  const subscribeAllVisible = () => {
    if (!connected) return;
    channels.forEach((c) => {
      if (subscribed[c.name] != null) return;
      wsClient.subscribe(c.name, 20);
      markSubscribed(c.name, 20);
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
          return (
            <li key={c.name} className="channel-row">
              <label className="row">
                <input
                  type="checkbox"
                  checked={isSub}
                  disabled={!connected}
                  onChange={() => toggle(c.name)}
                />
                <span>
                  {c.name} <span className="muted">{c.schema}</span>
                  {c.mock ? <span className="muted"> mock</span> : null}
                  {env?.stale ? <span className="stale-badge">stale</span> : null}
                  {env?.unsupported ? (
                    <span className="unsupported-badge">unsupported</span>
                  ) : null}
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
