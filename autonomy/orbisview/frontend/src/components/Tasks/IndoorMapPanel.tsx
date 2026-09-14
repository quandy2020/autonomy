import { useMemo, useRef, useState } from 'react';
import { zoneTypePalette } from '@/renderer/map2d/semanticZones';
import { useIndoorMapStore } from '@/store/indoorMapStore';
import { useLayerStore } from '@/store/layoutStore';

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

  const legend = useMemo(() => {
    const m = new Map<string, { count: number; fill: string }>();
    for (const z of zones) {
      const cur = m.get(z.zoneType);
      if (cur) cur.count += 1;
      else m.set(z.zoneType, { count: 1, fill: z.fill || zoneTypePalette(z.zoneType).fill });
    }
    return [...m.entries()].map(([type, v]) => ({ type, ...v }));
  }, [zones]);

  const loadText = (text: string) => {
    setErr(null);
    try {
      const json = JSON.parse(text) as Record<string, unknown>;
      setFromFixture(json);
    } catch (e) {
      setErr(e instanceof Error ? e.message : String(e));
    }
  };

  return (
    <div className="panel indoor-map-panel">
      <h3>Indoor Map</h3>
      <p className="hint">
        语义区 / 多楼层。图层 semantic · basemap 可在 Setting/Channels 切换。来源：{source}
      </p>

      <label className="mapping-check">
        <input
          type="checkbox"
          checked={semanticOn}
          onChange={(e) => setLayer('semantic', e.target.checked)}
        />
        显示语义区图层
      </label>

      <h4>楼层</h4>
      {floors.length === 0 ? (
        <p className="hint">暂无楼层（加载 fixture 或订阅 floors channel）</p>
      ) : (
        <ul className="indoor-floor-list">
          {floors.map((f) => (
            <li key={f.id}>
              <button
                type="button"
                className={f.id === activeFloorId ? 'active' : undefined}
                onClick={() => setActiveFloor(f.id)}
              >
                {f.name}
                <span className="muted"> · L{f.level}</span>
              </button>
            </li>
          ))}
        </ul>
      )}

      <h4>语义图例</h4>
      {legend.length === 0 ? (
        <p className="hint">暂无 zones</p>
      ) : (
        <ul className="indoor-legend">
          {legend.map((g) => (
            <li key={g.type}>
              <span className="indoor-swatch" style={{ background: g.fill }} />
              {g.type}
              <span className="muted"> ×{g.count}</span>
            </li>
          ))}
        </ul>
      )}

      <h4>静态 fixture</h4>
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
      <div className="row" style={{ gap: 8, flexWrap: 'wrap' }}>
        <button type="button" onClick={() => fileRef.current?.click()}>
          选择 JSON…
        </button>
        <button type="button" disabled={source !== 'static'} onClick={() => clearStatic()}>
          清除静态
        </button>
      </div>
      <textarea
        className="mapping-url-json"
        rows={5}
        value={paste}
        onChange={(e) => setPaste(e.target.value)}
        placeholder='{"floors":[...],"zones":[...],"active_floor_id":"F1"}'
      />
      <button type="button" disabled={!paste.trim()} onClick={() => loadText(paste)}>
        从粘贴加载
      </button>
      {err ? <p className="error">{err}</p> : null}
    </div>
  );
}
