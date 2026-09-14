import { useMemo, useRef, useState } from 'react';
import { zoneTypePalette } from '@/renderer/map2d/semanticZones';
import { useIndoorMapStore } from '@/store/indoorMapStore';
import { useLayerStore } from '@/store/layoutStore';

const SOURCE_LABEL: Record<string, string> = {
  none: '无数据',
  static: '静态',
  live: '实时',
};

/** Floors list, semantic legend, static fixture load. */
export function IndoorMapPanel() {
  const floors = useIndoorMapStore((s) => s.floors);
  const activeFloorId = useIndoorMapStore((s) => s.activeFloorId);
  const zones = useIndoorMapStore((s) => s.zones);
  const source = useIndoorMapStore((s) => s.source);
  const setActiveFloor = useIndoorMapStore((s) => s.setActiveFloor);
  const setFromFixture = useIndoorMapStore((s) => s.setFromFixture);
  const clearStatic = useIndoorMapStore((s) => s.clearStatic);
  const semanticOn = useLayerStore((s) => s.semantic);
  const setLayer = useLayerStore((s) => s.setLayer);

  const fileRef = useRef<HTMLInputElement>(null);
  const [paste, setPaste] = useState('');
  const [err, setErr] = useState<string | null>(null);
  const [ioOpen, setIoOpen] = useState(source === 'none');

  const legend = useMemo(() => {
    const m = new Map<string, { count: number; fill: string }>();
    for (const z of zones) {
      const cur = m.get(z.zoneType);
      if (cur) cur.count += 1;
      else m.set(z.zoneType, { count: 1, fill: z.fill || zoneTypePalette(z.zoneType).fill });
    }
    return [...m.entries()].map(([type, v]) => ({ type, ...v }));
  }, [zones]);

  const activeFloor = floors.find((f) => f.id === activeFloorId) ?? null;

  const loadText = (text: string) => {
    setErr(null);
    try {
      const json = JSON.parse(text) as Record<string, unknown>;
      setFromFixture(json);
      setIoOpen(false);
    } catch (e) {
      setErr(e instanceof Error ? e.message : String(e));
    }
  };

  return (
    <div className="panel indoor-map-panel">
      <div className="im-banner">
        <span className={`im-badge im-badge-${source}`}>{SOURCE_LABEL[source] ?? source}</span>
        <p className="hint im-banner-text">
          语义区 / 多楼层。图层可在 Setting · Channels 切换。
          {activeFloor ? (
            <>
              {' '}
              当前 <strong>{activeFloor.name}</strong>
            </>
          ) : null}
        </p>
      </div>

      <div className="im-stats" aria-label="概览">
        <div className="im-stat">
          <span className="im-stat-val">{floors.length}</span>
          <span className="im-stat-label">楼层</span>
        </div>
        <div className="im-stat">
          <span className="im-stat-val">{zones.length}</span>
          <span className="im-stat-label">语义区</span>
        </div>
        <div className="im-stat">
          <span className="im-stat-val">{legend.length}</span>
          <span className="im-stat-label">类型</span>
        </div>
      </div>

      <div className="im-toolbar">
        <label className={`im-chip${semanticOn ? ' on' : ''}`}>
          <input
            type="checkbox"
            checked={semanticOn}
            onChange={(e) => setLayer('semantic', e.target.checked)}
          />
          语义区图层
        </label>
      </div>

      <section className="im-section">
        <header className="im-section-head">
          <h4>楼层</h4>
          <span className="im-section-meta">{floors.length}</span>
        </header>
        {floors.length === 0 ? (
          <p className="im-empty">暂无楼层 · 加载 fixture 或订阅 floors channel</p>
        ) : (
          <ul className="im-floor-list">
            {floors.map((f) => {
              const active = f.id === activeFloorId;
              return (
                <li key={f.id}>
                  <button
                    type="button"
                    className={`im-floor-btn${active ? ' active' : ''}`}
                    onClick={() => setActiveFloor(f.id)}
                  >
                    <span className="im-floor-level">L{f.level}</span>
                    <span className="im-floor-main">
                      <span className="im-floor-name">{f.name}</span>
                      <span className="im-floor-id">{f.id}</span>
                    </span>
                    {active ? <span className="im-floor-active-tag">当前</span> : null}
                  </button>
                </li>
              );
            })}
          </ul>
        )}
      </section>

      <section className="im-section">
        <header className="im-section-head">
          <h4>语义图例</h4>
          <span className="im-section-meta">{legend.length}</span>
        </header>
        {legend.length === 0 ? (
          <p className="im-empty">暂无语义区</p>
        ) : (
          <ul className="im-legend">
            {legend.map((g) => (
              <li key={g.type}>
                <span className="im-swatch" style={{ background: g.fill }} />
                <span className="im-legend-type">{g.type}</span>
                <span className="im-legend-count">×{g.count}</span>
              </li>
            ))}
          </ul>
        )}
      </section>

      <div className="im-io">
        <button
          type="button"
          className="im-io-toggle"
          aria-expanded={ioOpen}
          onClick={() => setIoOpen((v) => !v)}
        >
          静态 fixture
          <span className="im-io-chevron">{ioOpen ? '▾' : '▸'}</span>
        </button>
        {ioOpen ? (
          <div className="im-io-body">
            <input
              ref={fileRef}
              type="file"
              accept="application/json,.json"
              hidden
              onChange={async (e) => {
                const f = e.target.files?.[0];
                if (!f) return;
                loadText(await f.text());
                e.target.value = '';
              }}
            />
            <div className="im-io-actions">
              <button type="button" onClick={() => fileRef.current?.click()}>
                选择 JSON
              </button>
              <button
                type="button"
                className="im-danger"
                disabled={source !== 'static'}
                onClick={() => clearStatic()}
              >
                清除静态
              </button>
            </div>
            <textarea
              className="im-paste"
              rows={4}
              value={paste}
              onChange={(e) => setPaste(e.target.value)}
              placeholder='{"floors":[...],"zones":[...],"active_floor_id":"F1"}'
            />
            <button
              type="button"
              className="im-paste-btn"
              disabled={!paste.trim()}
              onClick={() => loadText(paste)}
            >
              从粘贴加载
            </button>
            {err ? <p className="im-error">{err}</p> : null}
          </div>
        ) : null}
      </div>
    </div>
  );
}
