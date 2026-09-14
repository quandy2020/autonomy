import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
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
  collectMosaicIds,
  LAYOUT_PRESETS,
  type LayoutPresetId,
  type SidebarTab,
} from '@/store/layoutStore';
import { getPanel, listPanelsByCategory } from '@/components/registry';
import { allocPanelInstanceId, panelBaseId } from '@/components/panelId';
import {
  ModeSettingsPanel,
} from '@/components/Mode/ModePanels';
import { ChannelsSidebar } from '@/components/Channels/ChannelsSidebar';
import { SidebarTask } from '@/components/Sidebar/SidebarTask';
import { SidebarSetting } from '@/components/Sidebar/SidebarSetting';
import { emergencyStop } from '@/components/Teleop/emergencyStop';
import { SCHEMAS } from '@/store/websocket/types';
import { defaultWsUrl } from '@/config/parameters';
import { Icon, IconLabel, panelIcon, type IconName } from '@/components/icons';
import { usePanelOptsStore } from '@/store/panelOptsStore';
import { MappingVizEffects } from '@/components/MappingVizEffects';
import { IndoorMapEffects } from '@/components/IndoorMapEffects';

const SIDEBAR_NAV: { id: SidebarTab; label: string; icon: IconName }[] = [
  { id: 'panels', label: 'Panels', icon: 'panels' },
  { id: 'channels', label: 'Channels', icon: 'channels' },
  { id: 'task', label: 'Task', icon: 'task' },
  { id: 'setting', label: 'Setting', icon: 'setting' },
];
/** Offline mock channels (when backend --mock=true). */
const MOCK_CHANNELS = [
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
  '/orbisview/mock/semantic_zones',
  '/orbisview/mock/floors',
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

/** Autosim / stack channels preferred when --autolink=true. */
const AUTOSIM_CHANNELS = [
  '/odom',
  '/scan',
  '/map',
  '/tf',
  '/tf_static',
  '/footprint',
  '/overall/map',
  '/camera/rgb/image_raw',
  '/camera/depth/image_raw',
  '/camera/depth/points',
  '/camera/semantic/image_raw',
  '/points',
  '/imu',
];

function resubscribeTracked(): void {
  wsClient.listChannels();
  const { subscribed, markSubscribed, channels } = useDataStore.getState();
  const targets = { ...subscribed };
  const discovered = channels.map((c) => c.name);
  const preferLive = discovered.some((n) => !n.startsWith('/orbisview/mock/'));
  // Empty discovery must NOT fall back to mock — that creates Autolink readers
  // for /orbisview/mock/* while waiting for autosim.
  const seed = preferLive
    ? [...AUTOSIM_CHANNELS, ...discovered.filter((n) => !n.startsWith('/orbisview/mock/'))]
    : discovered.length > 0
      ? MOCK_CHANNELS
      : AUTOSIM_CHANNELS;
  for (const ch of seed) {
    if (targets[ch] == null) targets[ch] = 20;
  }
  // Drop stale mock subscriptions when live channels exist.
  if (preferLive) {
    for (const ch of Object.keys(targets)) {
      if (ch.startsWith('/orbisview/mock/')) delete targets[ch];
    }
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
  return <Comp panelId={id} />;
}

export function Orbisview() {
  const [url, setUrl] = useState(() => defaultWsUrl());
  const [topPanel, setTopPanel] = useState<null | 'mode' | 'layout'>(null);
  const layoutFileRef = useRef<HTMLInputElement>(null);
  const connectionState = useDataStore((s) => s.connectionState);
  const setConnectionState = useDataStore((s) => s.setConnectionState);
  const setChannels = useDataStore((s) => s.setChannels);
  const upsertEnvelope = useDataStore((s) => s.upsertEnvelope);
  const refreshStale = useDataStore((s) => s.refreshStale);
  const pushLog = useDataStore((s) => s.pushLog);
  const markSubscribed = useDataStore((s) => s.markSubscribed);
  const envelopes = useDataStore((s) => s.envelopes);

  const mosaic = useLayoutStore((s) => s.mosaic);
  const setMosaicRaw = useLayoutStore((s) => s.setMosaic);
  const setMosaic = useCallback(
    (node: MosaicNode<string> | null) => {
      setMosaicRaw(node);
      usePanelOptsStore.getState().prunePanels(collectMosaicIds(node));
    },
    [setMosaicRaw],
  );
  const catalogOpen = useLayoutStore((s) => s.catalogOpen);
  const setCatalogOpen = useLayoutStore((s) => s.setCatalogOpen);
  const sidebarTab = useLayoutStore((s) => s.sidebarTab);
  const setSidebarTab = useLayoutStore((s) => s.setSidebarTab);
  const applyLayoutPreset = useLayoutStore((s) => s.applyLayoutPreset);
  const exportLayoutJson = useLayoutStore((s) => s.exportLayoutJson);
  const importLayoutJson = useLayoutStore((s) => s.importLayoutJson);

  const applyPreset = (id: LayoutPresetId) => {
    applyLayoutPreset(id);
    usePanelOptsStore.getState().prunePanels(
      collectMosaicIds(useLayoutStore.getState().mosaic),
    );
    setTopPanel(null);
  };

  const saveLayoutFile = () => {
    const blob = new Blob([exportLayoutJson()], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `orbisview-layout-${new Date().toISOString().slice(0, 10)}.json`;
    a.click();
    URL.revokeObjectURL(url);
    setTopPanel(null);
  };

  const onLoadLayoutFile = (file: File | null) => {
    if (!file) return;
    void file.text().then((raw) => {
      const ok = importLayoutJson(raw);
      if (ok) {
        usePanelOptsStore.getState().prunePanels(
          collectMosaicIds(useLayoutStore.getState().mosaic),
        );
      } else {
        window.alert('布局文件无效或无法解析');
      }
      setTopPanel(null);
    });
  };
  useEffect(() => {
    if (!topPanel) return;
    const onKey = (e: KeyboardEvent) => {
      if (e.key === 'Escape') setTopPanel(null);
    };
    const onPointer = (e: MouseEvent) => {
      const t = e.target as Element | null;
      if (t?.closest?.('.topbar-pop')) return;
      setTopPanel(null);
    };
    window.addEventListener('keydown', onKey);
    window.addEventListener('mousedown', onPointer);
    return () => {
      window.removeEventListener('keydown', onKey);
      window.removeEventListener('mousedown', onPointer);
    };
  }, [topPanel]);

  useEffect(() => {
    const offMsg = wsClient.onMessage((msg) => {
      if (msg.op === 'channels') {
        setChannels(msg.channels);
        pushLog(`channels=${msg.channels.length}`);
        const { subscribed } = useDataStore.getState();
        const preferLive = msg.channels.some(
          (c) => !c.name.startsWith('/orbisview/mock/'),
        );
        for (const ch of msg.channels) {
          if (preferLive && ch.name.startsWith('/orbisview/mock/')) continue;
          if (!preferLive && !MOCK_CHANNELS.includes(ch.name)) continue;
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

  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      if (e.key !== ' ') return;
      const tag = (e.target as HTMLElement)?.tagName;
      if (tag === 'INPUT' || tag === 'TEXTAREA') return;
      e.preventDefault();
      emergencyStop();
    };
    window.addEventListener('keydown', onKey);
    return () => window.removeEventListener('keydown', onKey);
  }, []);

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
      const meta = getPanel(id);
      const title = meta?.title ?? panelBaseId(id);
      const suffix = id.includes('#') ? ` · ${id.slice(id.indexOf('#') + 1)}` : '';
      const label = `${title}${suffix}`;
      return (
        <MosaicWindow<string>
          path={path as never}
          title={label}
          renderToolbar={() => (
            <div className="mosaic-toolbar-row">
              <div className="mosaic-window-title ov-icon-label" title={label}>
                <Icon name={panelIcon(panelBaseId(id))} size={13} />
                <span className="mosaic-window-title-text">{label}</span>
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

  const addPanel = (typeId: string) => {
    const meta = getPanel(typeId);
    if (!meta) return;
    const cur = useLayoutStore.getState().mosaic;
    const existing = collectMosaicIds(cur);
    const id =
      meta.allowMultiple === true
        ? allocPanelInstanceId(meta.id, existing)
        : meta.id;
    if (!meta.allowMultiple && existing.includes(id)) return;
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
      <MappingVizEffects />
      <IndoorMapEffects />
      <header className="topbar">
        <div className="topbar-cluster topbar-brand-cluster">
          <strong className="brand">
            <Icon name="orbis" size={15} />
            <span className="brand-text">OrbisView</span>
          </strong>
          <span className="topbar-vsep" aria-hidden />
          <div className="topbar-conn">
            <input
              className="topbar-url"
              value={url}
              onChange={(e) => setUrl(e.target.value)}
              spellCheck={false}
              aria-label="WebSocket URL"
            />
            {connectionState === 'online' || connectionState === 'reconnecting' ? (
              <button
                type="button"
                className="topbar-btn"
                onClick={disconnect}
                title="Disconnect"
              >
                <IconLabel name="unplug" label="Disconnect" size={13} />
              </button>
            ) : (
              <button type="button" className="topbar-btn primary" onClick={connect} title="Connect">
                <IconLabel name="plug" label="Connect" size={13} />
              </button>
            )}
            <span className={`status-pill ${statusClass}`} title={connectionState}>
              <Icon name={statusIcon} size={11} />
              <span className="status-pill-text">{connectionState}</span>
            </span>
          </div>
        </div>

        <div className="topbar-cluster topbar-hud-cluster" aria-label="Vehicle status">
          <div className="hud-chip" title="Pose">
            <Icon name="pose" size={12} />
            <span className="hud-k">pose</span>
            <span className="hud-v">
              {pose
                ? `${pose.x.toFixed(2)}, ${pose.y.toFixed(2)}, ${(pose.yaw ?? 0).toFixed(2)}`
                : '—'}
            </span>
          </div>
          <div className="hud-chip" title="Twist">
            <Icon name="velocity" size={12} />
            <span className="hud-k">vel</span>
            <span className="hud-v">
              {twist ? `${twist.vx.toFixed(2)} / ${twist.wz.toFixed(2)}` : '—'}
            </span>
          </div>
          <div className="hud-chip" title="Chassis">
            <Icon name="chassis" size={12} />
            <span className="hud-k">mode</span>
            <span className="hud-v">
              {chassis?.driving_mode ?? '—'}
              {chassis?.gear ? ` ${chassis.gear}` : ''}
            </span>
          </div>
        </div>

        <div className="topbar-cluster topbar-actions">
          <div className="topbar-pop">
            <button
              type="button"
              className={topPanel === 'mode' ? 'topbar-btn active' : 'topbar-btn'}
              onClick={() => setTopPanel((v) => (v === 'mode' ? null : 'mode'))}
              title="Mode"
            >
              <IconLabel name="mode" label="Mode" size={13} />
            </button>
            {topPanel === 'mode' ? (
              <div className="topbar-pop-card topbar-mode-card" role="dialog" aria-label="Mode">
                <ModeSettingsPanel />
              </div>
            ) : null}
          </div>
          <div className="topbar-pop">
            <button
              type="button"
              className={topPanel === 'layout' ? 'topbar-btn active' : 'topbar-btn'}
              onClick={() => setTopPanel((v) => (v === 'layout' ? null : 'layout'))}
              title="Layout"
            >
              <IconLabel name="layout" label="Layout" size={13} />
            </button>
            {topPanel === 'layout' ? (
              <div className="topbar-pop-card topbar-layout-card" role="dialog" aria-label="Layout">
                <h3 className="sidebar-section-title">
                  <IconLabel name="layout" label="Layout" size={13} />
                </h3>
                <div className="layout-menu">
                  {(Object.keys(LAYOUT_PRESETS) as LayoutPresetId[]).map((id) => {
                    const p = LAYOUT_PRESETS[id];
                    return (
                      <button
                        key={id}
                        type="button"
                        className="layout-menu-item"
                        onClick={() => applyPreset(id)}
                      >
                        <Icon name="preset" size={14} />
                        <span className="layout-menu-text">
                          <strong>{p.label}</strong>
                          <em>{p.hint}</em>
                        </span>
                      </button>
                    );
                  })}
                  <hr className="sep" />
                  <button type="button" className="layout-menu-item" onClick={saveLayoutFile}>
                    <Icon name="dump" size={14} />
                    <span className="layout-menu-text">
                      <strong>保存布局</strong>
                      <em>导出当前 mosaic 为 JSON</em>
                    </span>
                  </button>
                  <button
                    type="button"
                    className="layout-menu-item"
                    onClick={() => layoutFileRef.current?.click()}
                  >
                    <Icon name="resources" size={14} />
                    <span className="layout-menu-text">
                      <strong>加载布局</strong>
                      <em>从 JSON 文件恢复</em>
                    </span>
                  </button>
                  <input
                    ref={layoutFileRef}
                    type="file"
                    accept="application/json,.json"
                    hidden
                    onChange={(e) => {
                      onLoadLayoutFile(e.target.files?.[0] ?? null);
                      e.target.value = '';
                    }}
                  />
                  <hr className="sep" />
                  <button
                    type="button"
                    className="layout-menu-item"
                    onClick={() => {
                      setCatalogOpen(!catalogOpen);
                      setTopPanel(null);
                    }}
                  >
                    <Icon name="sidebar" size={14} />
                    <span className="layout-menu-text">
                      <strong>{catalogOpen ? '隐藏侧栏' : '显示侧栏'}</strong>
                      <em>Panels / Channels / Task / Setting</em>
                    </span>
                  </button>
                </div>
              </div>
            ) : null}
          </div>
          <button
            type="button"
            className="estop topbar-estop"
            onClick={emergencyStop}
            disabled={connectionState !== 'online'}
            title="Emergency stop (Space)"
          >
            E-STOP
          </button>
        </div>
      </header>

      <div className="body ops-body">
        <aside className={`ops-nav${catalogOpen ? ' is-open' : ' is-collapsed'}`}>
          <nav className="ops-rail" aria-label="Sidebar">
            {SIDEBAR_NAV.map(({ id, label, icon }) => (
              <button
                key={id}
                type="button"
                className={
                  catalogOpen && sidebarTab === id ? 'ops-rail-btn active' : 'ops-rail-btn'
                }
                onClick={() => setSidebarTab(id)}
                title={label}
                aria-pressed={catalogOpen && sidebarTab === id}
              >
                <Icon name={icon} size={16} />
                <span>{label}</span>
              </button>
            ))}
            <button
              type="button"
              className="ops-rail-btn ops-rail-toggle"
              onClick={() => setCatalogOpen(!catalogOpen)}
              title={catalogOpen ? 'Hide sidebar' : 'Show sidebar'}
            >
              <Icon name="sidebar" size={16} />
              <span>{catalogOpen ? 'Hide' : 'Show'}</span>
            </button>
          </nav>

          {catalogOpen ? (
            <div className="ops-pane sidebar">
              <div className="sidebar-body">
                {sidebarTab === 'channels' ? <ChannelsSidebar /> : null}
                {sidebarTab === 'task' ? <SidebarTask /> : null}
                {sidebarTab === 'setting' ? <SidebarSetting /> : null}
                {sidebarTab === 'panels' ? (
                  <div className="panels-sidebar">
                    <h3 className="sidebar-section-title">
                      <IconLabel name="panels" label="Panels" size={13} />
                    </h3>
                    <p className="hint sidebar-hint">Click to add a panel to the mosaic.</p>
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
                  </div>
                ) : null}
              </div>
            </div>
          ) : null}
        </aside>

        <main className="main mosaic-main">
          <Mosaic<string>
            renderTile={renderTile}
            value={mosaic}
            onChange={(node: MosaicNode<string> | null) => setMosaic(node)}
            className="mosaic-blueprint-theme mosaic-host"
            zeroStateView={<div className="panel">Add a panel from Sidebar → Panels</div>}
          />
        </main>
      </div>
    </div>
  );
}
