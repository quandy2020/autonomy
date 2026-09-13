import { useCallback, useEffect, useMemo, useState } from 'react';
import {
  Mosaic,
  MosaicWindow,
  ExpandButton,
  RemoveButton,
  type MosaicNode,
} from 'react-mosaic-component';
import 'react-mosaic-component/react-mosaic-component.css';
import { wsClient } from '@/store/websocket/client';
import { useDataStore } from '@/store/dataStore';
import {
  useLayoutStore,
  useLayerStore,
  collectMosaicIds,
  type LayerKey,
} from '@/store/layoutStore';
import { getPanel, listPanelsByCategory } from '@/components/registry';
import { TeleopPanel } from '@/components/Teleop/TeleopPanel';
import {
  ModeSettingsPanel,
  ResourceManagerPanel,
} from '@/components/Mode/ModePanels';
import { SCHEMAS } from '@/store/websocket/types';
import { DEFAULT_WS_URL } from '@/config/parameters';
import { Icon, IconLabel, layerIcon, panelIcon } from '@/components/icons';

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
  const [url, setUrl] = useState(DEFAULT_WS_URL);
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

  const statusIcon =
    connectionState === 'online'
      ? 'statusOnline'
      : connectionState === 'reconnecting' || connectionState === 'connecting'
        ? 'statusWarn'
        : 'statusOffline';

  const renderTile = useCallback(
    (id: string, path: unknown) => {
      const title = getPanel(id)?.title ?? id;
      return (
        <MosaicWindow<string>
          path={path as never}
          title={title}
          renderToolbar={() => (
            <div className="mosaic-toolbar-row">
              <div className="mosaic-window-title ov-icon-label" title={title}>
                <Icon name={panelIcon(id)} size={13} />
                <span className="mosaic-window-title-text">{title}</span>
              </div>
              <div className="mosaic-window-controls ov-mosaic-controls">
                <ExpandButton />
                <RemoveButton />
              </div>
            </div>
          )}
        >
          <div className="mosaic-panel-body">
            <PanelTile id={id} />
          </div>
        </MosaicWindow>
      );
    },
    [],
  );

  const addPanel = (id: string) => {
    const cur = useLayoutStore.getState().mosaic;
    if (!cur) {
      setMosaic(id);
      return;
    }
    if (collectMosaicIds(cur).includes(id)) return;
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
        <strong className="brand">
          <Icon name="orbis" size={16} />
          OrbisView
        </strong>
        <input value={url} onChange={(e) => setUrl(e.target.value)} size={22} />
        {connectionState === 'online' || connectionState === 'reconnecting' ? (
          <button type="button" className="btn-icon" onClick={disconnect}>
            <IconLabel name="unplug" label="Disconnect" size={14} />
          </button>
        ) : (
          <button type="button" className="btn-icon" onClick={connect}>
            <IconLabel name="plug" label="Connect" size={14} />
          </button>
        )}
        <span className={`status-pill ${statusClass}`}>
          <Icon name={statusIcon} size={12} />
          {connectionState}
        </span>
        <span className="topbar-hud muted">
          <Icon name="pose" size={12} />
          {pose
            ? `${pose.x.toFixed(2)}, ${pose.y.toFixed(2)}, ${(pose.yaw ?? 0).toFixed(2)}`
            : '—'}
        </span>
        <span className="topbar-hud muted">
          <Icon name="velocity" size={12} />
          {twist ? `vx ${twist.vx.toFixed(2)}  wz ${twist.wz.toFixed(2)}` : '—'}
        </span>
        <span className="topbar-hud muted">
          <Icon name="chassis" size={12} />
          {chassis?.driving_mode ?? 'mode —'} {chassis?.gear ?? ''}
        </span>
        <button type="button" className="btn-icon" onClick={() => setCatalogOpen(!catalogOpen)}>
          <IconLabel name="catalog" label="Catalog" size={14} />
        </button>
        <button type="button" className="btn-icon" onClick={resetGroundPreset}>
          <IconLabel name="preset" label="Ground" size={14} />
        </button>
      </header>

      <div className="body ops-body">
        {catalogOpen ? (
          <aside className="sidebar ops-sidebar">
            <div className="row" style={{ gap: 4, flexWrap: 'wrap', marginBottom: 8 }}>
              {(
                [
                  ['mode', 'Mode', 'mode'],
                  ['panels', 'Add Panel', 'panels'],
                  ['resources', 'Resources', 'resources'],
                  ['layers', 'Layers', 'layers'],
                ] as const
              ).map(([id, label, icon]) => (
                <button
                  key={id}
                  type="button"
                  className={sidebarTab === id ? 'tab active btn-icon' : 'tab btn-icon'}
                  onClick={() => setSidebarTab(id)}
                >
                  <IconLabel name={icon} label={label} size={13} />
                </button>
              ))}
            </div>
            {sidebarTab === 'mode' ? <ModeSettingsPanel /> : null}
            {sidebarTab === 'panels' ? (
              <>
                <h3>Add Panel</h3>
                <div className="catalog-groups">
                  {listPanelsByCategory().map(({ category, panels }) => (
                    <section key={category.id} className="catalog-group">
                      <h4 className="catalog-group-title">
                        <IconLabel name={category.icon} label={category.label} size={13} />
                      </h4>
                      {panels.map((p) => (
                        <button
                          key={p.id}
                          type="button"
                          className="link catalog-item btn-icon"
                          onClick={() => addPanel(p.id)}
                        >
                          <IconLabel name={panelIcon(p.id)} label={p.title} size={14} />
                          <Icon name="plus" size={12} className="catalog-add" />
                        </button>
                      ))}
                    </section>
                  ))}
                </div>
              </>
            ) : null}
            {sidebarTab === 'resources' ? <ResourceManagerPanel /> : null}
            {sidebarTab === 'layers' ? (
              <>
                <h3>
                  <IconLabel name="layers" label="Layers" size={13} />
                </h3>
                {LAYER_KEYS.map((k) => (
                  <label key={k} className="row layer-toggle">
                    <input
                      type="checkbox"
                      checked={layers[k]}
                      onChange={(e) => layers.setLayer(k, e.target.checked)}
                    />
                    <Icon name={layerIcon(k)} size={13} />
                    <span>{k}</span>
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
            className={bottomMode === 'teleop' ? 'tab active btn-icon' : 'tab btn-icon'}
            onClick={() => setBottomMode('teleop')}
          >
            <IconLabel name="teleop" label="Teleop" size={14} />
          </button>
          <button
            type="button"
            className={bottomMode === 'pnc' ? 'tab active btn-icon' : 'tab btn-icon'}
            onClick={() => setBottomMode('pnc')}
          >
            <IconLabel name="pnc" label="PNC" size={14} />
          </button>
          <button
            type="button"
            className={bottomMode === 'ops' ? 'tab active btn-icon' : 'tab btn-icon'}
            onClick={() => setBottomMode('ops')}
          >
            <IconLabel name="ops" label="Ops" size={14} />
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
                className="btn-icon"
                disabled={connectionState !== 'online'}
                onClick={() =>
                  wsClient.send({ op: 'dump_snapshot', path: '/tmp/orbisview_dump.json' })
                }
              >
                <IconLabel name="dump" label="Dump" size={13} />
              </button>
              <button
                type="button"
                className="btn-icon"
                disabled={connectionState !== 'online'}
                onClick={() => wsClient.send({ op: 'clear_sim' })}
              >
                <IconLabel name="clear" label="Clear" size={13} />
              </button>
              <button
                type="button"
                className="btn-icon"
                disabled={connectionState !== 'online'}
                onClick={() =>
                  wsClient.send({
                    op: 'playback_start',
                    path: '/tmp/orbisview_record.jsonl',
                    speed: 1,
                  })
                }
              >
                <IconLabel name="play" label="Play" size={13} />
              </button>
              <button
                type="button"
                className="btn-icon"
                disabled={connectionState !== 'online'}
                onClick={() => wsClient.send({ op: 'playback_pause', paused: true })}
              >
                <IconLabel name="pause" label="Pause" size={13} />
              </button>
              <button
                type="button"
                className="btn-icon"
                disabled={connectionState !== 'online'}
                onClick={() => wsClient.send({ op: 'playback_stop' })}
              >
                <IconLabel name="stop" label="Stop" size={13} />
              </button>
              <span className="hint">DV+ bottom ops · Dump/Clear/Play</span>
            </div>
          ) : null}
        </div>
      </footer>
    </div>
  );
}
