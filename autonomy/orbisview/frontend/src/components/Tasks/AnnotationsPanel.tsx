import { useRef, useState } from 'react';
import { useAnnotationStore } from '@/store/annotationStore';
import { useLayerStore } from '@/store/layoutStore';
import {
  poiKindColor,
  type DrawShapeKind,
  type PoiKind,
} from '@/renderer/map2d/annotations';

const POI_KIND_LABEL: Record<PoiKind, string> = {
  charger: '充电桩',
  elevator: '电梯',
  custom: '自定义',
};

const DRAW_KIND_LABEL: Record<DrawShapeKind, string> = {
  polygon: '多边形',
  polyline: '折线',
};

/** Local POI / draw annotation list, import/export. */
export function AnnotationsPanel() {
  const pois = useAnnotationStore((s) => s.pois);
  const shapes = useAnnotationStore((s) => s.shapes);
  const selectedId = useAnnotationStore((s) => s.selectedId);
  const poiDefaultKind = useAnnotationStore((s) => s.poiDefaultKind);
  const drawDefaultKind = useAnnotationStore((s) => s.drawDefaultKind);
  const setPoiDefaultKind = useAnnotationStore((s) => s.setPoiDefaultKind);
  const setDrawDefaultKind = useAnnotationStore((s) => s.setDrawDefaultKind);
  const setSelected = useAnnotationStore((s) => s.setSelected);
  const updatePoi = useAnnotationStore((s) => s.updatePoi);
  const removePoi = useAnnotationStore((s) => s.removePoi);
  const updateShape = useAnnotationStore((s) => s.updateShape);
  const removeShape = useAnnotationStore((s) => s.removeShape);
  const importJson = useAnnotationStore((s) => s.importJson);
  const exportJson = useAnnotationStore((s) => s.exportJson);
  const clearAll = useAnnotationStore((s) => s.clearAll);

  const poiOn = useLayerStore((s) => s.poi);
  const drawOn = useLayerStore((s) => s.draw);
  const setLayer = useLayerStore((s) => s.setLayer);

  const fileRef = useRef<HTMLInputElement>(null);
  const [paste, setPaste] = useState('');
  const [err, setErr] = useState<string | null>(null);
  const [tab, setTab] = useState<'poi' | 'draw'>('poi');
  const [ioOpen, setIoOpen] = useState(false);

  const onImportText = (text: string) => {
    setErr(null);
    try {
      importJson(text);
    } catch (e) {
      setErr(e instanceof Error ? e.message : String(e));
    }
  };

  const onExport = () => {
    const blob = new Blob([exportJson()], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = 'orbisview-annotations.json';
    a.click();
    URL.revokeObjectURL(url);
  };

  return (
    <div className="panel annotations-panel">
      <div className="ann-banner">
        <span className="ann-badge">本地</span>
        <p className="hint ann-banner-text">
          标注仅存于本机浏览器。用地图工具栏 <strong>POI</strong> / <strong>绘制</strong> 落点。
        </p>
      </div>

      <div className="ann-tabs" role="tablist" aria-label="标注类型">
        <button
          type="button"
          role="tab"
          aria-selected={tab === 'poi'}
          className={tab === 'poi' ? 'active' : undefined}
          onClick={() => setTab('poi')}
        >
          POI
          <span className="ann-count">{pois.length}</span>
        </button>
        <button
          type="button"
          role="tab"
          aria-selected={tab === 'draw'}
          className={tab === 'draw' ? 'active' : undefined}
          onClick={() => setTab('draw')}
        >
          形状
          <span className="ann-count">{shapes.length}</span>
        </button>
      </div>

      <div className="ann-toolbar">
        <label className={`ann-chip${poiOn ? ' on' : ''}`}>
          <input
            type="checkbox"
            checked={poiOn}
            onChange={(e) => setLayer('poi', e.target.checked)}
          />
          POI 层
        </label>
        <label className={`ann-chip${drawOn ? ' on' : ''}`}>
          <input
            type="checkbox"
            checked={drawOn}
            onChange={(e) => setLayer('draw', e.target.checked)}
          />
          绘制层
        </label>
        {tab === 'poi' ? (
          <label className="ann-default">
            <span>默认</span>
            <select
              value={poiDefaultKind}
              onChange={(e) => setPoiDefaultKind(e.target.value as PoiKind)}
            >
              {(Object.keys(POI_KIND_LABEL) as PoiKind[]).map((k) => (
                <option key={k} value={k}>
                  {POI_KIND_LABEL[k]}
                </option>
              ))}
            </select>
          </label>
        ) : (
          <label className="ann-default">
            <span>默认</span>
            <select
              value={drawDefaultKind}
              onChange={(e) => setDrawDefaultKind(e.target.value as DrawShapeKind)}
            >
              {(Object.keys(DRAW_KIND_LABEL) as DrawShapeKind[]).map((k) => (
                <option key={k} value={k}>
                  {DRAW_KIND_LABEL[k]}
                </option>
              ))}
            </select>
          </label>
        )}
      </div>

      {tab === 'poi' ? (
        <ul className="ann-list" role="listbox" aria-label="POI 列表">
          {pois.length === 0 ? (
            <li className="ann-empty">暂无 POI · 选中地图工具栏「POI」后单击落点</li>
          ) : (
            pois.map((p) => {
              const selected = p.id === selectedId;
              const color = p.color ?? poiKindColor(p.kind);
              return (
                <li key={p.id} className={`ann-item${selected ? ' selected' : ''}`}>
                  <button
                    type="button"
                    className="ann-item-head"
                    role="option"
                    aria-selected={selected}
                    onClick={() => setSelected(selected ? null : p.id)}
                  >
                    <span className="ann-swatch" style={{ background: color }} />
                    <span className="ann-item-main">
                      <span className="ann-item-title">{p.label || POI_KIND_LABEL[p.kind]}</span>
                      <span className="ann-item-meta">
                        {POI_KIND_LABEL[p.kind]}
                        <span className="ann-coord">
                          ({p.x.toFixed(2)}, {p.y.toFixed(2)})
                        </span>
                      </span>
                    </span>
                  </button>
                  {selected ? (
                    <div className="ann-item-edit">
                      <label>
                        名称
                        <input
                          value={p.label ?? ''}
                          placeholder="可选标签"
                          onChange={(e) => updatePoi(p.id, { label: e.target.value })}
                        />
                      </label>
                      <label>
                        类型
                        <select
                          value={p.kind}
                          onChange={(e) => updatePoi(p.id, { kind: e.target.value as PoiKind })}
                        >
                          {(Object.keys(POI_KIND_LABEL) as PoiKind[]).map((k) => (
                            <option key={k} value={k}>
                              {POI_KIND_LABEL[k]}
                            </option>
                          ))}
                        </select>
                      </label>
                      <button
                        type="button"
                        className="ann-danger"
                        onClick={() => removePoi(p.id)}
                      >
                        删除
                      </button>
                    </div>
                  ) : null}
                </li>
              );
            })
          )}
        </ul>
      ) : (
        <ul className="ann-list" role="listbox" aria-label="形状列表">
          {shapes.length === 0 ? (
            <li className="ann-empty">暂无形状 · 选中「绘制」后加点，双击/右击结束</li>
          ) : (
            shapes.map((s) => {
              const selected = s.id === selectedId;
              return (
                <li key={s.id} className={`ann-item${selected ? ' selected' : ''}`}>
                  <button
                    type="button"
                    className="ann-item-head"
                    role="option"
                    aria-selected={selected}
                    onClick={() => setSelected(selected ? null : s.id)}
                  >
                    <span
                      className={`ann-swatch ann-swatch-${s.kind}`}
                      aria-hidden
                    />
                    <span className="ann-item-main">
                      <span className="ann-item-title">
                        {s.label || DRAW_KIND_LABEL[s.kind]}
                      </span>
                      <span className="ann-item-meta">
                        {DRAW_KIND_LABEL[s.kind]}
                        <span className="ann-coord">{s.points.length} 点</span>
                      </span>
                    </span>
                  </button>
                  {selected ? (
                    <div className="ann-item-edit">
                      <label>
                        名称
                        <input
                          value={s.label ?? ''}
                          placeholder="可选标签"
                          onChange={(e) => updateShape(s.id, { label: e.target.value })}
                        />
                      </label>
                      <button
                        type="button"
                        className="ann-danger"
                        onClick={() => removeShape(s.id)}
                      >
                        删除
                      </button>
                    </div>
                  ) : null}
                </li>
              );
            })
          )}
        </ul>
      )}

      <div className="ann-io">
        <button
          type="button"
          className="ann-io-toggle"
          aria-expanded={ioOpen}
          onClick={() => setIoOpen((v) => !v)}
        >
          导入 / 导出
          <span className="ann-io-chevron">{ioOpen ? '▾' : '▸'}</span>
        </button>
        {ioOpen ? (
          <div className="ann-io-body">
            <input
              ref={fileRef}
              type="file"
              accept="application/json,.json"
              hidden
              onChange={async (e) => {
                const f = e.target.files?.[0];
                if (!f) return;
                onImportText(await f.text());
                e.target.value = '';
              }}
            />
            <div className="ann-io-actions">
              <button type="button" onClick={() => fileRef.current?.click()}>
                导入 JSON
              </button>
              <button type="button" onClick={onExport}>
                导出
              </button>
              <button
                type="button"
                className="ann-danger"
                onClick={() => {
                  if (window.confirm('清除全部标注？')) clearAll();
                }}
              >
                全部清除
              </button>
            </div>
            <textarea
              className="ann-paste"
              rows={3}
              value={paste}
              onChange={(e) => setPaste(e.target.value)}
              placeholder='粘贴 JSON：{"version":1,"pois":[],"shapes":[]}'
            />
            <button
              type="button"
              className="ann-paste-btn"
              disabled={!paste.trim()}
              onClick={() => onImportText(paste)}
            >
              从粘贴导入
            </button>
            {err ? <p className="ann-error">{err}</p> : null}
          </div>
        ) : null}
      </div>
    </div>
  );
}
