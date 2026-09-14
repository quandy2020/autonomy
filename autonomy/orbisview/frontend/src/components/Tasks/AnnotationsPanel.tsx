import { useRef, useState } from 'react';
import { useAnnotationStore } from '@/store/annotationStore';
import { useLayerStore } from '@/store/layoutStore';
import type { DrawShapeKind, PoiKind } from '@/renderer/map2d/annotations';

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
      <h3>Annotations</h3>
      <p className="hint">POI / 绘制仅保存在本机浏览器；可用工具栏 poi、draw 工具操作。</p>

      <div className="mapping-mode-row" role="group">
        <button type="button" className={tab === 'poi' ? 'active' : undefined} onClick={() => setTab('poi')}>
          POI ({pois.length})
        </button>
        <button type="button" className={tab === 'draw' ? 'active' : undefined} onClick={() => setTab('draw')}>
          形状 ({shapes.length})
        </button>
      </div>

      <label className="mapping-check">
        <input type="checkbox" checked={poiOn} onChange={(e) => setLayer('poi', e.target.checked)} />
        显示 POI 层
      </label>
      <label className="mapping-check">
        <input type="checkbox" checked={drawOn} onChange={(e) => setLayer('draw', e.target.checked)} />
        显示绘制层
      </label>

      {tab === 'poi' ? (
        <>
          <div className="mapping-field-row">
            <label>默认类型</label>
            <select
              value={poiDefaultKind}
              onChange={(e) => setPoiDefaultKind(e.target.value as PoiKind)}
            >
              <option value="charger">charger</option>
              <option value="elevator">elevator</option>
              <option value="custom">custom</option>
            </select>
          </div>
          <ul className="indoor-floor-list">
            {pois.map((p) => (
              <li key={p.id}>
                <button
                  type="button"
                  className={p.id === selectedId ? 'active' : undefined}
                  onClick={() => setSelected(p.id)}
                >
                  {p.label || p.kind} · ({p.x.toFixed(2)}, {p.y.toFixed(2)})
                </button>
                {p.id === selectedId ? (
                  <div className="mapping-field-row">
                    <input
                      value={p.label ?? ''}
                      placeholder="label"
                      onChange={(e) => updatePoi(p.id, { label: e.target.value })}
                    />
                    <select
                      value={p.kind}
                      onChange={(e) => updatePoi(p.id, { kind: e.target.value as PoiKind })}
                    >
                      <option value="charger">charger</option>
                      <option value="elevator">elevator</option>
                      <option value="custom">custom</option>
                    </select>
                    <button type="button" onClick={() => removePoi(p.id)}>
                      删除
                    </button>
                  </div>
                ) : null}
              </li>
            ))}
          </ul>
        </>
      ) : (
        <>
          <div className="mapping-field-row">
            <label>默认形状</label>
            <select
              value={drawDefaultKind}
              onChange={(e) => setDrawDefaultKind(e.target.value as DrawShapeKind)}
            >
              <option value="polygon">polygon</option>
              <option value="polyline">polyline</option>
            </select>
          </div>
          <ul className="indoor-floor-list">
            {shapes.map((s) => (
              <li key={s.id}>
                <button
                  type="button"
                  className={s.id === selectedId ? 'active' : undefined}
                  onClick={() => setSelected(s.id)}
                >
                  {s.label || s.kind} · {s.points.length} pts
                </button>
                {s.id === selectedId ? (
                  <div className="mapping-field-row">
                    <input
                      value={s.label ?? ''}
                      placeholder="label"
                      onChange={(e) => updateShape(s.id, { label: e.target.value })}
                    />
                    <button type="button" onClick={() => removeShape(s.id)}>
                      删除
                    </button>
                  </div>
                ) : null}
              </li>
            ))}
          </ul>
        </>
      )}

      <h4>导入 / 导出</h4>
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
      <div className="row" style={{ gap: 8, flexWrap: 'wrap' }}>
        <button type="button" onClick={() => fileRef.current?.click()}>
          导入 JSON…
        </button>
        <button type="button" onClick={onExport}>
          导出
        </button>
        <button
          type="button"
          onClick={() => {
            if (window.confirm('清除全部标注？')) clearAll();
          }}
        >
          全部清除
        </button>
      </div>
      <textarea
        className="mapping-url-json"
        rows={4}
        value={paste}
        onChange={(e) => setPaste(e.target.value)}
        placeholder='{"version":1,"pois":[],"shapes":[]}'
      />
      <button type="button" disabled={!paste.trim()} onClick={() => onImportText(paste)}>
        从粘贴导入
      </button>
      {err ? <p className="error">{err}</p> : null}
    </div>
  );
}
