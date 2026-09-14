import { useEffect, useMemo, useRef, useState } from 'react';
import { Icon } from '@/components/icons';
import {
  DISPLAY_TYPE_DEFS,
  MSG_PACKAGES,
  channelMatchesType,
  getDisplayTypeDef,
  inferTypeFromChannel,
  messagesForPackage,
  resolveDisplayType,
  type DisplayPropDef,
  type DisplayTypeDef,
  type PropValue,
} from '@/components/Channels/displayTypes';
import {
  formatPayloadJson,
  isMetaMessage,
  isMetaTypeId,
  msgIcon,
  packageIcon,
} from '@/components/Channels/msgVisual';
import { isImageDisplayType, openImagePanel } from '@/components/Channels/openImagePanel';
import { useDataStore } from '@/store/dataStore';
import { useDisplayStore } from '@/store/displayStore';
import { wsClient } from '@/store/websocket/client';

function PropEditor({
  def,
  value,
  topicOptions,
  onChange,
}: {
  def: DisplayPropDef;
  value: PropValue;
  topicOptions: { name: string; schema: string }[];
  onChange: (v: PropValue) => void;
}) {
  if (def.kind === 'boolean') {
    return (
      <input
        type="checkbox"
        checked={!!value}
        onChange={(e) => onChange(e.target.checked)}
      />
    );
  }
  if (def.kind === 'number') {
    return (
      <input
        className="ch-hz-input"
        type="number"
        min={def.min}
        max={def.max}
        step={def.step ?? 1}
        value={Number(value)}
        onChange={(e) => onChange(Number(e.target.value))}
      />
    );
  }
  if (def.kind === 'select') {
    return (
      <select
        className="ch-select"
        value={String(value)}
        onChange={(e) => onChange(e.target.value)}
      >
        {(def.options ?? []).map((o) => (
          <option key={o} value={o}>
            {o}
          </option>
        ))}
      </select>
    );
  }
  if (def.kind === 'color') {
    return (
      <input
        type="color"
        value={String(value || '#4fc3f7')}
        onChange={(e) => onChange(e.target.value)}
      />
    );
  }
  if (def.kind === 'topic') {
    return (
      <select
        className="ch-select ch-topic-select"
        value={String(value || '')}
        onChange={(e) => onChange(e.target.value)}
        title={String(value || '')}
      >
        <option value="">— channel —</option>
        {topicOptions.map((t) => (
          <option key={t.name} value={t.name} title={t.name}>
            {t.name}
          </option>
        ))}
      </select>
    );
  }
  return (
    <input
      className="ch-text-input"
      type="text"
      value={String(value ?? '')}
      onChange={(e) => onChange(e.target.value)}
    />
  );
}

function TypePropertyTree({
  def,
  props,
  topicOptions,
  hz,
  latencyMs,
  stale,
  payload,
  onProp,
}: {
  def: DisplayTypeDef;
  props: Record<string, PropValue>;
  topicOptions: { name: string; schema: string }[];
  hz?: number;
  latencyMs?: number;
  stale?: boolean;
  payload?: unknown;
  onProp: (key: string, value: PropValue) => void;
}) {
  const meta = isMetaMessage(def.package, def.message);
  const groups = useMemo(() => {
    const map = new Map<string, DisplayPropDef[]>();
    for (const p of def.props) {
      if (p.key === 'enabled') continue; // header checkbox already covers this
      if (meta && p.group !== 'Status' && p.group !== 'Channel' && p.group !== 'Topic') continue;
      const arr = map.get(p.group) ?? [];
      arr.push(p);
      map.set(p.group, arr);
    }
    // Prefer Channel / Style first for scanning.
    const order = ['Channel', 'Topic', 'Style', 'Status', 'Image', 'Frames', 'History', 'Filter'];
    return [...map.entries()].sort(
      (a, b) => (order.indexOf(a[0]) + 1 || 99) - (order.indexOf(b[0]) + 1 || 99),
    );
  }, [def.props, meta]);

  const [jsonOpen, setJsonOpen] = useState(true);

  return (
    <div className="ch-props">
      <div className="ch-metrics" title="Rate">
        <span className="ch-metric">
          <em>Hz</em>
          {hz != null && Number.isFinite(hz) ? hz.toFixed(1) : '—'}
        </span>
        <span className="ch-metric">
          <em>lat</em>
          {latencyMs != null ? `${latencyMs.toFixed(0)}ms` : '—'}
        </span>
        <span className={`ch-metric ${stale ? 'is-stale' : 'is-ok'}`}>
          {stale ? 'stale' : 'live'}
        </span>
      </div>

      {groups.map(([group, defs]) => (
        <div key={group} className="ch-props-block">
          <div className="ch-props-label">{group}</div>
          <div className="ch-props-rows">
            {defs.map((p) => (
              <label key={p.key} className="ch-prop">
                <span className="ch-prop-k">{p.label}</span>
                <span className="ch-prop-v">
                  <PropEditor
                    def={p}
                    value={props[p.key] ?? p.defaultValue}
                    topicOptions={topicOptions}
                    onChange={(v) => onProp(p.key, v)}
                  />
                </span>
              </label>
            ))}
          </div>
        </div>
      ))}

      {meta ? (
        <div className="ch-props-block">
          <button
            type="button"
            className="ch-props-label is-btn"
            onClick={() => setJsonOpen((v) => !v)}
          >
            <span className="ch-caret">{jsonOpen ? '▾' : '▸'}</span>
            JSON
            <span className="ch-json-badge">meta</span>
          </button>
          {jsonOpen ? <pre className="ch-json-view">{formatPayloadJson(payload)}</pre> : null}
        </div>
      ) : null}

      <div className="ch-type-foot" title={def.typeId}>
        {def.typeId}
      </div>
    </div>
  );
}

function AddDisplayDialog({
  open,
  onClose,
  onAdd,
  onAddMany,
  existingChannels,
}: {
  open: boolean;
  onClose: () => void;
  onAdd: (packageName: string, message: string, channel?: string) => void;
  onAddMany: (items: { packageName: string; message: string; channel: string }[]) => void;
  existingChannels: Set<string>;
}) {
  const channels = useDataStore((s) => s.channels);
  const connected = useDataStore((s) => s.connected);
  const [mode, setMode] = useState<'type' | 'live'>('type');
  const [pkg, setPkg] = useState('sensor_msgs');
  const [filter, setFilter] = useState('');
  const [picked, setPicked] = useState<Record<string, boolean>>({});
  const pkgListRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    if (!open) return;
    setPicked({});
    setFilter('');
    if (connected) wsClient.listChannels();
  }, [open, connected]);

  useEffect(() => {
    if (!open || mode !== 'type') return;
    const root = pkgListRef.current;
    if (!root) return;
    const active = root.querySelector<HTMLElement>('button.active');
    active?.scrollIntoView({ block: 'nearest' });
  }, [open, mode, pkg]);

  const messages = useMemo(() => {
    const q = filter.trim().toLowerCase();
    return messagesForPackage(pkg).filter(
      (m) => !q || m.message.toLowerCase().includes(q) || m.package.toLowerCase().includes(q),
    );
  }, [pkg, filter]);

  const curated = useMemo(
    () => new Set(DISPLAY_TYPE_DEFS.filter((d) => d.package === pkg).map((d) => d.message)),
    [pkg],
  );

  const liveChannels = useMemo(() => {
    const q = filter.trim().toLowerCase();
    const rows = [...channels].sort((a, b) => {
      const aw = a.has_writer === b.has_writer ? 0 : a.has_writer ? -1 : 1;
      if (aw !== 0) return aw;
      return a.name.localeCompare(b.name);
    });
    return rows.filter((c) => {
      if (!q) return true;
      const hay = `${c.name} ${c.schema} ${c.msg_type}`.toLowerCase();
      return hay.includes(q);
    });
  }, [channels, filter]);

  const selectedLive = useMemo(
    () => liveChannels.filter((c) => picked[c.name] && !existingChannels.has(c.name)),
    [liveChannels, picked, existingChannels],
  );

  if (!open) return null;

  return (
    <div className="ch-add-modal" role="dialog" aria-label="Add display">
      <div className="ch-add-card">
        <div className="ch-add-head">
          <strong>Add display</strong>
          <button type="button" className="channels-icon-btn" onClick={onClose} title="Close">
            ×
          </button>
        </div>

        <div className="ch-add-mode">
          <button
            type="button"
            className={mode === 'type' ? 'active' : ''}
            onClick={() => setMode('type')}
          >
            By type
          </button>
          <button
            type="button"
            className={mode === 'live' ? 'active' : ''}
            onClick={() => setMode('live')}
          >
            Live channels
            {channels.length ? <span className="ch-add-count">{channels.length}</span> : null}
          </button>
        </div>

        {mode === 'type' ? (
          <>
            <p className="muted ch-add-hint">From automsgs/proto/msgs — pick a message type</p>
            <div className="ch-add-body">
              <div className="ch-add-packages" ref={pkgListRef}>
                {MSG_PACKAGES.map((p) => (
                  <button
                    key={p}
                    type="button"
                    className={pkg === p ? 'active' : ''}
                    onClick={() => setPkg(p)}
                  >
                    <Icon name={packageIcon(p)} size={13} />
                    <span>{p}</span>
                  </button>
                ))}
              </div>
              <div className="ch-add-messages">
                <input
                  className="ch-add-filter"
                  value={filter}
                  onChange={(e) => setFilter(e.target.value)}
                  placeholder="Filter message…"
                />
                <div className="ch-add-msg-list">
                  {messages.map((m) => {
                    const def = resolveDisplayType(m.package, m.message);
                    const meta = isMetaMessage(m.package, m.message);
                    const icon = msgIcon(m.package, m.message);
                    return (
                      <button
                        key={`${m.package}/${m.message}`}
                        type="button"
                        className="ch-add-msg-item"
                        onClick={() => {
                          onAdd(m.package, m.message);
                          onClose();
                        }}
                      >
                        <span className="ch-add-msg-icon" aria-hidden>
                          <Icon name={icon} size={16} />
                        </span>
                        <span className="ch-add-msg-text">
                          <span className="ch-add-msg-name">
                            {m.message}
                            {meta ? <span className="ch-json-badge">json</span> : null}
                            {!meta && curated.has(m.message) ? (
                              <span className="ch-viz-badge">viz</span>
                            ) : null}
                          </span>
                          <span className="muted ch-add-msg-type">{def.typeId}</span>
                        </span>
                      </button>
                    );
                  })}
                  {!messages.length ? <p className="muted">No messages</p> : null}
                </div>
              </div>
            </div>
          </>
        ) : (
          <>
            <p className="muted ch-add-hint">
              Publishing channels — check to add as displays
              {!connected ? ' (offline)' : ''}
            </p>
            <div className="ch-add-live">
              <div className="ch-add-live-toolbar">
                <input
                  className="ch-add-filter"
                  value={filter}
                  onChange={(e) => setFilter(e.target.value)}
                  placeholder="Filter channel / schema…"
                />
                <button
                  type="button"
                  className="channels-icon-btn"
                  title="Refresh"
                  disabled={!connected}
                  onClick={() => wsClient.listChannels()}
                >
                  <Icon name="mapping" size={13} />
                </button>
              </div>
              <div className="ch-add-msg-list ch-add-live-list">
                {liveChannels.map((c) => {
                  const inferred = inferTypeFromChannel(c);
                  const def = resolveDisplayType(inferred.package, inferred.message);
                  const meta = isMetaMessage(inferred.package, inferred.message);
                  const icon = msgIcon(inferred.package, inferred.message);
                  const already = existingChannels.has(c.name);
                  const checked = already || !!picked[c.name];
                  return (
                    <label
                      key={c.name}
                      className={`ch-add-msg-item ch-add-live-item${already ? ' is-added' : ''}`}
                    >
                      <input
                        type="checkbox"
                        checked={checked}
                        disabled={already}
                        onChange={(e) =>
                          setPicked((s) => ({ ...s, [c.name]: e.target.checked }))
                        }
                      />
                      <span className="ch-add-msg-icon" aria-hidden>
                        <Icon name={icon} size={16} />
                      </span>
                      <span className="ch-add-msg-text">
                        <span className="ch-add-msg-name">
                          {c.name}
                          {c.has_writer ? <span className="ch-live-badge">live</span> : null}
                          {already ? <span className="ch-added-badge">added</span> : null}
                          {meta ? <span className="ch-json-badge">json</span> : null}
                          {!meta && DISPLAY_TYPE_DEFS.some((d) => d.typeId === def.typeId) ? (
                            <span className="ch-viz-badge">viz</span>
                          ) : null}
                        </span>
                        <span className="muted ch-add-msg-type">
                          {def.typeId}
                          {c.schema && c.schema !== c.msg_type ? ` · ${c.schema}` : ''}
                        </span>
                      </span>
                    </label>
                  );
                })}
                {!liveChannels.length ? (
                  <p className="muted">
                    {connected ? 'No channels yet — wait for publishers.' : 'Connect to list channels.'}
                  </p>
                ) : null}
              </div>
              <div className="ch-add-live-foot">
                <span className="muted">
                  {selectedLive.length} selected
                  {existingChannels.size ? ` · ${existingChannels.size} already added` : ''}
                </span>
                <button
                  type="button"
                  className="ch-add-btn"
                  disabled={!selectedLive.length}
                  onClick={() => {
                    onAddMany(
                      selectedLive.map((c) => {
                        const t = inferTypeFromChannel(c);
                        return {
                          packageName: t.package,
                          message: t.message,
                          channel: c.name,
                        };
                      }),
                    );
                    onClose();
                  }}
                >
                  <Icon name="plus" size={13} />
                  Add selected
                </button>
              </div>
            </div>
          </>
        )}
      </div>
    </div>
  );
}

/** RViz2 Displays–style: Add by automsgs type, on-demand property tree. */
export function ChannelsSidebar() {
  const channels = useDataStore((s) => s.channels);
  const envelopes = useDataStore((s) => s.envelopes);
  const markSubscribed = useDataStore((s) => s.markSubscribed);
  const markUnsubscribed = useDataStore((s) => s.markUnsubscribed);
  const connected = useDataStore((s) => s.connected);

  const displays = useDisplayStore((s) => s.displays);
  const selectedId = useDisplayStore((s) => s.selectedId);
  const addDisplay = useDisplayStore((s) => s.addDisplay);
  const removeDisplay = useDisplayStore((s) => s.removeDisplay);
  const selectDisplay = useDisplayStore((s) => s.selectDisplay);
  const setEnabled = useDisplayStore((s) => s.setEnabled);
  const setChannel = useDisplayStore((s) => s.setChannel);
  const setMaxHz = useDisplayStore((s) => s.setMaxHz);
  const setProp = useDisplayStore((s) => s.setProp);

  const [addOpen, setAddOpen] = useState(false);
  const [hzByChannel, setHzByChannel] = useState<Record<string, number>>({});
  const [latencyByChannel, setLatencyByChannel] = useState<Record<string, number>>({});
  const [expanded, setExpanded] = useState<Record<string, boolean>>({});

  useEffect(() => {
    if (!connected) return;
    const off = wsClient.onMessage((msg) => {
      if (msg.op !== 'channel_stats') return;
      const hz: Record<string, number> = {};
      const lat: Record<string, number> = {};
      for (const row of msg.channels ?? []) {
        hz[row.name] = row.hz;
        lat[row.name] = row.latency_ms;
      }
      setHzByChannel(hz);
      setLatencyByChannel(lat);
    });
    wsClient.listChannels();
    wsClient.send({ op: 'channel_stats' });
    const timer = window.setInterval(() => wsClient.send({ op: 'channel_stats' }), 1000);
    return () => {
      off();
      window.clearInterval(timer);
    };
  }, [connected]);

  // Sync subscriptions from enabled displays with a topic.
  useEffect(() => {
    if (!connected) return;
    const want = new Map<string, number>();
    for (const d of displays) {
      if (!d.enabled || !d.channel) continue;
      want.set(d.channel, d.maxHz || 20);
    }
    const { subscribed } = useDataStore.getState();
    for (const [ch, hz] of want) {
      if (subscribed[ch] !== hz) {
        wsClient.subscribe(ch, hz);
        markSubscribed(ch, hz);
      }
    }
    for (const ch of Object.keys(subscribed)) {
      if (!want.has(ch) && ch.startsWith('/orbisview/')) {
        // Keep auto mock subs from Orbisview bootstrap; only manage display-driven extras carefully.
        // Unsubscribe only if this channel is exclusively from a disabled display? Skip auto-unsub of defaults.
      }
    }
  }, [connected, displays, markSubscribed]);

  const onToggleEnabled = (id: string, enabled: boolean) => {
    const d = displays.find((x) => x.id === id);
    setEnabled(id, enabled);
    if (!d?.channel || !connected) return;
    if (enabled) {
      wsClient.subscribe(d.channel, d.maxHz || 20);
      markSubscribed(d.channel, d.maxHz || 20);
    } else {
      const stillNeeded = displays.some(
        (x) => x.id !== id && x.enabled && x.channel === d.channel,
      );
      if (!stillNeeded) {
        wsClient.unsubscribe(d.channel);
        markUnsubscribed(d.channel);
      }
    }
  };

  return (
    <div className="channels-sidebar">
      <div className="channels-toolbar">
        <div className="ch-toolbar-group" role="group" aria-label="Channels actions">
          <button type="button" className="ch-toolbar-main" onClick={() => setAddOpen(true)}>
            <Icon name="plus" size={13} />
            Add
          </button>
          <button
            type="button"
            className="ch-toolbar-side"
            disabled={!connected}
            title="Refresh channels"
            onClick={() => {
              wsClient.listChannels();
              wsClient.send({ op: 'channel_stats' });
            }}
          >
            <Icon name="mapping" size={13} />
          </button>
        </div>
      </div>

      <div className="channels-meta muted">
        {displays.length} display{displays.length === 1 ? '' : 's'} · click Add to pick type
      </div>

      {!displays.length ? (
        <p className="muted channels-empty">
          No displays yet. Click <strong>Add</strong> and choose a type from automsgs.
        </p>
      ) : null}

      <div className="ch-tree">
        {displays.map((d) => {
          const def = getDisplayTypeDef(d.typeId) ?? resolveDisplayType(
            d.typeId.split('/')[0] ?? 'std_msgs',
            d.typeId.split('/')[1] ?? d.typeId,
          );
          const open = expanded[d.id] ?? selectedId === d.id;
          const topicOptions = channels.filter((c) => channelMatchesType(c, def));
          const hz = d.channel ? hzByChannel[d.channel] : undefined;
          const lat = d.channel ? latencyByChannel[d.channel] : undefined;
          const env = d.channel ? envelopes[d.channel] : undefined;
          const title = String(d.props.name || def.label);
          const icon = msgIcon(def.package, def.message);
          const meta = isMetaTypeId(def.typeId);

          return (
            <div key={d.id} className={`ch-leaf${d.enabled ? ' is-sub' : ''}${open ? ' is-open' : ''}`}>
              <div className="ch-leaf-row">
                <button
                  type="button"
                  className="ch-caret-btn"
                  onClick={() => {
                    setExpanded((s) => ({ ...s, [d.id]: !open }));
                    selectDisplay(d.id);
                  }}
                >
                  <span className="ch-caret">{open ? '▾' : '▸'}</span>
                </button>
                <label className="ch-check">
                  <input
                    type="checkbox"
                    checked={d.enabled}
                    onChange={(e) => onToggleEnabled(d.id, e.target.checked)}
                  />
                </label>
                <button
                  type="button"
                  className="ch-leaf-main"
                  onClick={() => {
                    setExpanded((s) => ({ ...s, [d.id]: true }));
                    selectDisplay(d.id);
                    if (isImageDisplayType(d.typeId)) {
                      openImagePanel(d.channel || null);
                    }
                  }}
                >
                  <Icon name={icon} size={13} className="ch-leaf-icon" />
                  <span className="ch-leaf-name">
                    {title}
                    {meta ? <span className="ch-json-badge">json</span> : null}
                  </span>
                  <span className="ch-leaf-type muted" title={d.channel || undefined}>
                    {def.typeId}
                    {d.channel ? ` · ${d.channel}` : ''}
                  </span>
                </button>
                <span className={`ch-hz ${d.enabled && d.channel ? 'on' : 'off'}`}>
                  {hz != null && Number.isFinite(hz) ? hz.toFixed(1) : '—'}
                  <span className="ch-hz-unit">Hz</span>
                </span>
                <button
                  type="button"
                  className="ch-remove-btn"
                  title="Remove"
                  onClick={() => {
                    if (d.channel && d.enabled) onToggleEnabled(d.id, false);
                    removeDisplay(d.id);
                  }}
                >
                  ×
                </button>
              </div>
              {open ? (
                <TypePropertyTree
                  def={def}
                  props={d.props}
                  topicOptions={topicOptions.length ? topicOptions : channels}
                  hz={hz}
                  latencyMs={lat}
                  stale={env?.stale}
                  payload={env?.payload}
                  onProp={(key, value) => {
                    setProp(d.id, key, value);
                    if (key === 'topic' && typeof value === 'string') {
                      setChannel(d.id, value);
                      if (d.enabled && value && connected) {
                        wsClient.subscribe(value, d.maxHz || 20);
                        markSubscribed(value, d.maxHz || 20);
                      }
                      if (isImageDisplayType(d.typeId)) {
                        openImagePanel(value || null);
                      }
                    }
                    if (key === 'maxHz' && typeof value === 'number') {
                      setMaxHz(d.id, value);
                      if (d.enabled && d.channel && connected) {
                        wsClient.subscribe(d.channel, value);
                        markSubscribed(d.channel, value);
                      }
                    }
                    if (key === 'enabled' && typeof value === 'boolean') {
                      onToggleEnabled(d.id, value);
                    }
                  }}
                />
              ) : null}
            </div>
          );
        })}
      </div>

      <AddDisplayDialog
        open={addOpen}
        onClose={() => setAddOpen(false)}
        existingChannels={new Set(displays.map((d) => d.channel).filter(Boolean))}
        onAdd={(packageName, message, channel) => {
          const id = addDisplay(packageName, message, channel);
          setExpanded((s) => ({ ...s, [id]: true }));
          if (channel && connected) {
            wsClient.subscribe(channel, 20);
            markSubscribed(channel, 20);
          }
          const typeId = `${packageName}/${message}`;
          if (isImageDisplayType(typeId)) {
            openImagePanel(channel || null);
          }
        }}
        onAddMany={(items) => {
          const nextExpanded: Record<string, boolean> = {};
          for (const it of items) {
            const id = addDisplay(it.packageName, it.message, it.channel);
            nextExpanded[id] = true;
            if (it.channel && connected) {
              wsClient.subscribe(it.channel, 20);
              markSubscribed(it.channel, 20);
            }
            const typeId = `${it.packageName}/${it.message}`;
            if (isImageDisplayType(typeId)) {
              openImagePanel(it.channel || null);
            }
          }
          setExpanded((s) => ({ ...s, ...nextExpanded }));
        }}
      />
    </div>
  );
}
