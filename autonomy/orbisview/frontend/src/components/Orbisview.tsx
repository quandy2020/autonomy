import { useCallback, useEffect, useMemo, useState } from 'react';
import { Mosaic, MosaicWindow, type MosaicNode } from 'react-mosaic-component';
import 'react-mosaic-component/react-mosaic-component.css';
import { wsClient } from '@/store/websocket/client';
import { useDataStore } from '@/store/dataStore';
import {
  useLayoutStore,
  useLayerStore,
  type LayerKey,
} from '@/store/layoutStore';
import { getPanel, listPanels } from '@/components/registry';
import { TeleopPanel } from '@/components/Teleop/TeleopPanel';
import {
  ModeSettingsPanel,
  ResourceManagerPanel,
} from '@/components/Mode/ModePanels';
import { SCHEMAS } from '@/store/websocket/types';

const DEFAULT_CHANNELS = [
  '/orbisview/mock/pose',
  '/orbisview/mock/path',
  '/orbisview/mock/map',
  '/orbisview/mock/costmap',
  '/orbisview/mock/footprint',
  '/orbisview/mock/tf',
  '/orbisview/mock/laser',
  '/orbisview/mock/image',
  '/orbisview/mock/pointcloud',
  '/orbisview/mock/depth',
  '/orbisview/mock/exploration',
  '/orbisview/mock/navigation',
  '/orbisview/mock/mapping',
  '/orbisview/mock/twist',
  '/orbisview/mock/chassis',
  '/orbisview/mock/obstacles',
  '/orbisview/mock/world',
  '/orbisview/mock/route',
  '/orbisview/mock/vector_map',
  '/orbisview/mock/prediction',
  '/orbisview/mock/planning',
  '/orbisview/mock/hmi',
  '/orbisview/mock/components',
];

const LAYER_KEYS: LayerKey[] = [
  'grid',
  'map',
  'costmap',
  'vectormap',
  'path',
  'robot',
  'footprint',
  'obstacles',
  'prediction',
  'laser',
  'tf',
  'pointcloud',
  'image',
  'depth',
];

function resubscribeTracked(): void {
  wsClient.listChannels();
  const { subscribed, markSubscribed } = useDataStore.getState();
  const targets = { ...subscribed };
  for (const ch of DEFAULT_CHANNELS) {
    if (targets[ch] == null) targets[ch] = 20;
  }
  Object.entries(targets).forEach(([ch, hz]) => {
    wsClient.subscribe(ch, hz);
    markSubscribed(ch, hz);
  });
}

function asPayload<T>(env: { payload?: unknown } | undefined): T | null {
  if (!env?.payload || typeof env.payload !== 'object') return null;
  return env.payload as T;
}

function PanelTile({ id }: { id: string }) {
  const Comp = getPanel(id)?.component;
  if (!Comp) return <div className="panel">Unknown panel: {id}</div>;
  return <Comp />;
}

export function Orbisview() {
  const [url, setUrl] = useState('ws://127.0.0.1:8766/ws');
  const connectionState = useDataStore((s) => s.connectionState);
  const setConnectionState = useDataStore((s) => s.setConnectionState);
  const setChannels = useDataStore((s) => s.setChannels);
  const upsertEnvelope = useDataStore((s) => s.upsertEnvelope);
  const refreshStale = useDataStore((s) => s.refreshStale);
  const pushLog = useDataStore((s) => s.pushLog);
  const markSubscribed = useDataStore((s) => s.markSubscribed);
  const envelopes = useDataStore((s) => s.envelopes);

  const mosaic = useLayoutStore((s) => s.mosaic);
  const setMosaic = useLayoutStore((s) => s.setMosaic);
  const bottomMode = useLayoutStore((s) => s.bottomMode);
  const setBottomMode = useLayoutStore((s) => s.setBottomMode);
  const catalogOpen = useLayoutStore((s) => s.catalogOpen);
  const setCatalogOpen = useLayoutStore((s) => s.setCatalogOpen);
  const sidebarTab = useLayoutStore((s) => s.sidebarTab);
  const setSidebarTab = useLayoutStore((s) => s.setSidebarTab);
  const resetGroundPreset = useLayoutStore((s) => s.resetGroundPreset);
  const layers = useLayerStore();

  useEffect(() => {
    const offMsg = wsClient.onMessage((msg) => {
      if (msg.op === 'channels') {
        setChannels(msg.channels);
        pushLog(`channels=${msg.channels.length}`);
        const { subscribed } = useDataStore.getState();
        for (const ch of msg.channels) {
          if (!DEFAULT_CHANNELS.includes(ch.name)) continue;
          if (subscribed[ch.name] != null) continue;
          wsClient.subscribe(ch.name, 20);
          markSubscribed(ch.name, 20);
        }
      } else if (msg.op === 'envelope') {
        upsertEnvelope(msg);
      } else if (msg.op === 'subscribed') {
        markSubscribed(msg.channel, msg.max_hz);
      } else if (msg.op === 'error') {
        pushLog(`error: ${msg.message}`);
      } else if (msg.op === 'status') {
        pushLog(`status clients=${msg.clients} dropped=${msg.dropped_frames}`);
      }
    });
    const offConn = wsClient.onConnection((state) => {
      setConnectionState(state);
      if (state === 'online') {
        resubscribeTracked();
        pushLog('ws online');
      } else if (state === 'reconnecting') {
        pushLog('ws reconnecting…');
      }
    });
    const staleTimer = window.setInterval(() => refreshStale(), 500);
    return () => {
      offMsg();
      offConn();
      window.clearInterval(staleTimer);
    };
  }, [setChannels, upsertEnvelope, refreshStale, pushLog, setConnectionState, markSubscribed]);

  const pose = useMemo(() => {
    const e = Object.values(envelopes).find((x) => x.schema === SCHEMAS.Pose2D);
    return asPayload<{ x: number; y: number; yaw?: number }>(e);
  }, [envelopes]);

  const twist = useMemo(() => {
    const e = Object.values(envelopes).find((x) => x.schema === SCHEMAS.Twist2D);
    return asPayload<{ vx: number; wz: number }>(e);
  }, [envelopes]);

  const chassis = useMemo(() => {
    const e = Object.values(envelopes).find((x) => x.schema === SCHEMAS.ChassisState);
    return asPayload<{ driving_mode?: string; gear?: string }>(e);
  }, [envelopes]);

  const connect = () => wsClient.connect(url);
  const disconnect = () => wsClient.close();
  const PncMini = getPanel('pnc')?.component;

  const statusClass =
    connectionState === 'online'
      ? 'ok'
      : connectionState === 'reconnecting' || connectionState === 'connecting'
        ? 'warn'
        : 'bad';

  const renderTile = useCallback(
    (id: string, path: unknown) => (
      <MosaicWindow<string>
        path={path as never}
        title={getPanel(id)?.title ?? id}
        toolbarControls={<div />}
      >
        <div className="mosaic-panel-body">
          <PanelTile id={id} />
        </div>
      </MosaicWindow>
    ),
    [],
  );

  const addPanel = (id: string) => {
    const cur = useLayoutStore.getState().mosaic;
    if (!cur) {
      setMosaic(id);
      return;
    }
    setMosaic({
      type: 'split',
      direction: 'row',
      children: [cur, id],
      splitPercentages: [70, 30],
    });
  };

  return (
    <div className="app ops-shell">
      <header className="topbar">
        <strong>OrbisView</strong>
        <input value={url} onChange={(e) => setUrl(e.target.value)} size={22} />
        {connectionState === 'online' || connectionState === 'reconnecting' ? (
          <button type="button" onClick={disconnect}>
            Disconnect
          </button>
        ) : (
          <button type="button" onClick={connect}>
            Connect
          </button>
        )}
        <span className={statusClass}>{connectionState}</span>
        <span className="topbar-hud muted">
          {pose
            ? `pose ${pose.x.toFixed(2)},${pose.y.toFixed(2)},${(pose.yaw ?? 0).toFixed(2)}`
            : 'pose —'}
        </span>
        <span className="topbar-hud muted">
          {twist
            ? `vx ${twist.vx.toFixed(2)} wz ${twist.wz.toFixed(2)}`
            : 'twist —'}
        </span>
        <span className="topbar-hud muted">
          {chassis?.driving_mode ?? 'mode —'} {chassis?.gear ?? ''}
        </span>
        <button type="button" onClick={() => setCatalogOpen(!catalogOpen)}>
          Catalog
        </button>
        <button type="button" onClick={resetGroundPreset}>
          Ground preset
        </button>
      </header>

      <div className="body ops-body">
        {catalogOpen ? (
          <aside className="sidebar ops-sidebar">
            <div className="row" style={{ gap: 4, flexWrap: 'wrap', marginBottom: 8 }}>
              {(
                [
                  ['mode', 'Mode'],
                  ['panels', 'Add Panel'],
                  ['resources', 'Resources'],
                  ['layers', 'Layers'],
                ] as const
              ).map(([id, label]) => (
                <button
                  key={id}
                  type="button"
                  className={sidebarTab === id ? 'tab active' : 'tab'}
                  onClick={() => setSidebarTab(id)}
                >
                  {label}
                </button>
              ))}
            </div>
            {sidebarTab === 'mode' ? <ModeSettingsPanel /> : null}
            {sidebarTab === 'panels' ? (
              <>
                <h3>Add Panel</h3>
                {listPanels().map((p) => (
                  <button
                    key={p.id}
                    type="button"
                    className="link catalog-item"
                    onClick={() => addPanel(p.id)}
                  >
                    + {p.title}
                  </button>
                ))}
              </>
            ) : null}
            {sidebarTab === 'resources' ? <ResourceManagerPanel /> : null}
            {sidebarTab === 'layers' ? (
              <>
                <h3>Layers</h3>
                {LAYER_KEYS.map((k) => (
                  <label key={k} className="row">
                    <input
                      type="checkbox"
                      checked={layers[k]}
                      onChange={(e) => layers.setLayer(k, e.target.checked)}
                    />
                    {k}
                  </label>
                ))}
              </>
            ) : null}
          </aside>
        ) : null}

        <main className="main mosaic-main">
          <Mosaic<string>
            renderTile={renderTile}
            value={mosaic}
            onChange={(node: MosaicNode<string> | null) => setMosaic(node)}
            className="mosaic-blueprint-theme mosaic-host"
            zeroStateView={<div className="panel">Add a panel from Catalog</div>}
          />
        </main>
      </div>

      <footer className="bottom-bar">
        <div className="bottom-mode">
          <button
            type="button"
            className={bottomMode === 'teleop' ? 'tab active' : 'tab'}
            onClick={() => setBottomMode('teleop')}
          >
            Teleop
          </button>
          <button
            type="button"
            className={bottomMode === 'pnc' ? 'tab active' : 'tab'}
            onClick={() => setBottomMode('pnc')}
          >
            PNC
          </button>
          <button
            type="button"
            className={bottomMode === 'ops' ? 'tab active' : 'tab'}
            onClick={() => setBottomMode('ops')}
          >
            Ops
          </button>
        </div>
        <div className="bottom-content">
          {bottomMode === 'teleop' ? (
            <TeleopPanel />
          ) : bottomMode === 'pnc' && PncMini ? (
            <div className="pnc-mini">
              <PncMini />
            </div>
          ) : bottomMode === 'ops' ? (
            <div className="row" style={{ gap: 8, flexWrap: 'wrap' }}>
              <button
                type="button"
                disabled={connectionState !== 'online'}
                onClick={() =>
                  wsClient.send({ op: 'dump_snapshot', path: '/tmp/orbisview_dump.json' })
                }
              >
                Dump
              </button>
              <button
                type="button"
                disabled={connectionState !== 'online'}
                onClick={() => wsClient.send({ op: 'clear_sim' })}
              >
                Clear
              </button>
              <button
                type="button"
                disabled={connectionState !== 'online'}
                onClick={() =>
                  wsClient.send({
                    op: 'playback_start',
                    path: '/tmp/orbisview_record.jsonl',
                    speed: 1,
                  })
                }
              >
                Play
              </button>
              <button
                type="button"
                disabled={connectionState !== 'online'}
                onClick={() => wsClient.send({ op: 'playback_pause', paused: true })}
              >
                Pause
              </button>
              <button
                type="button"
                disabled={connectionState !== 'online'}
                onClick={() => wsClient.send({ op: 'playback_stop' })}
              >
                Stop
              </button>
              <span className="hint">DV+ bottom ops · Dump/Clear/Play</span>
            </div>
          ) : null}
        </div>
      </footer>
    </div>
  );
}
