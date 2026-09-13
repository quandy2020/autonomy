import { useEffect, useState } from 'react';
import { wsClient } from '@/store/websocket/client';
import { useDataStore } from '@/store/dataStore';
import { loadFrontendPluginManifests } from '@/plugins/frontendLoader';

type PluginRow = {
  id: string;
  kind: string;
  title: string;
  version: string;
  enabled: boolean;
  source?: string;
  path?: string;
};

export function PluginsPanel() {
  const connected = useDataStore((s) => s.connected);
  const [plugins, setPlugins] = useState<PluginRow[]>([]);
  const [failures, setFailures] = useState<string[]>([]);
  const [hostStatus, setHostStatus] = useState('');
  const [path, setPath] = useState('');
  const [scanDir, setScanDir] = useState('');
  const [feNote, setFeNote] = useState('');

  useEffect(() => {
    if (!connected) return;
    const off = wsClient.onMessage((msg) => {
      const m = msg as {
        op?: string;
        plugins?: PluginRow[];
        failures?: string[];
      };
      if (m.op === 'plugins') {
        setPlugins(m.plugins ?? []);
        setFailures(m.failures ?? []);
      } else if (m.op === 'plugin_host') {
        setHostStatus(JSON.stringify(msg, null, 2));
      }
    });
    wsClient.send({ op: 'list_plugins' });
    wsClient.send({ op: 'plugin_host_status' });
    return off;
  }, [connected]);

  useEffect(() => {
    loadFrontendPluginManifests('/plugins/manifest.json')
      .then((n) => setFeNote(n > 0 ? `Frontend manifests loaded: ${n}` : 'No frontend manifests'))
      .catch((e) => setFeNote(String(e)));
  }, []);

  return (
    <div className="panel">
      <h3>Plugin Host</h3>
      <p style={{ opacity: 0.8 }}>{feNote}</p>
      <label className="row">
        Load .so/.dylib
        <input
          value={path}
          onChange={(e) => setPath(e.target.value)}
          size={48}
          placeholder="/path/to/orbisview_example_hello.so"
        />
      </label>
      <div className="row" style={{ gap: 8, marginTop: 8, flexWrap: 'wrap' }}>
        <button
          type="button"
          onClick={() => wsClient.send({ op: 'plugins_load', path })}
          disabled={!path}
        >
          Load
        </button>
        <button type="button" onClick={() => wsClient.send({ op: 'list_plugins' })}>
          Refresh
        </button>
        <button type="button" onClick={() => wsClient.send({ op: 'plugin_host_status' })}>
          Host status
        </button>
      </div>
      <label className="row" style={{ marginTop: 8 }}>
        Scan directory
        <input
          value={scanDir}
          onChange={(e) => setScanDir(e.target.value)}
          size={48}
          placeholder="build-orbisview/lib/orbisview/plugins"
        />
      </label>
      <button
        type="button"
        style={{ marginTop: 8 }}
        onClick={() => wsClient.send({ op: 'plugins_scan', path: scanDir })}
        disabled={!scanDir}
      >
        Scan & load
      </button>

      <table style={{ marginTop: 16, width: '100%', borderCollapse: 'collapse' }}>
        <thead>
          <tr>
            <th align="left">id</th>
            <th align="left">kind</th>
            <th align="left">source</th>
            <th align="left">version</th>
            <th />
          </tr>
        </thead>
        <tbody>
          {plugins.map((p) => (
            <tr key={p.id}>
              <td>{p.title || p.id}</td>
              <td>{p.kind}</td>
              <td>{p.source ?? '—'}</td>
              <td>{p.version}</td>
              <td>
                {p.source === 'dynamic' ? (
                  <span className="row" style={{ gap: 4 }}>
                    <button
                      type="button"
                      onClick={() => wsClient.send({ op: 'plugins_reload', id: p.id })}
                    >
                      Reload
                    </button>
                    <button
                      type="button"
                      onClick={() => wsClient.send({ op: 'plugins_unload', id: p.id })}
                    >
                      Unload
                    </button>
                  </span>
                ) : null}
              </td>
            </tr>
          ))}
        </tbody>
      </table>
      {failures.length > 0 ? (
        <pre style={{ marginTop: 12, color: '#e57373' }}>{failures.join('\n')}</pre>
      ) : null}
      {hostStatus ? <pre style={{ marginTop: 12 }}>{hostStatus}</pre> : null}
    </div>
  );
}
