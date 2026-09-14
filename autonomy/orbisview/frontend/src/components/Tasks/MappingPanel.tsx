import { useMemo, useRef, useState } from 'react';
import {
  basemapFromDemoFrame,
  sortMappingFrameFiles,
  type MappingBasemapMeta,
} from '@/renderer/map2d/mappingDemo';
import {
  parseStaticSlamSidecar,
  probeImageSize,
} from '@/renderer/map2d/staticSlam';
import { useDataStore } from '@/store/dataStore';
import { useLayerStore } from '@/store/layoutStore';
import { useMappingVizStore } from '@/store/mappingVizStore';
import { useStaticSlamStore } from '@/store/staticSlamStore';
import { SCHEMAS } from '@/store/websocket/types';

function isMetaFile(f: File): boolean {
  const n = f.name.toLowerCase();
  return n.endsWith('.yaml') || n.endsWith('.yml') || n.endsWith('.json');
}

function isImageFile(f: File): boolean {
  const n = f.name.toLowerCase();
  return (
    n.endsWith('.png') ||
    n.endsWith('.jpg') ||
    n.endsWith('.jpeg') ||
    n.endsWith('.bmp') ||
    n.endsWith('.pgm') ||
    n.endsWith('.webp') ||
    f.type.startsWith('image/')
  );
}

/** Full mapping viz controls: demo replay + live Mapping / mirror. */
export function MappingPanel() {
  const mode = useMappingVizStore((s) => s.mode);
  const demo = useMappingVizStore((s) => s.demo);
  const mirror = useMappingVizStore((s) => s.live.mirrorMapToBasemap);
  const setMode = useMappingVizStore((s) => s.setMode);
  const setIntervalMs = useMappingVizStore((s) => s.setIntervalMs);
  const setMirrorMapToBasemap = useMappingVizStore((s) => s.setMirrorMapToBasemap);
  const loadDemoFrames = useMappingVizStore((s) => s.loadDemoFrames);
  const clearDemoFrames = useMappingVizStore((s) => s.clearDemoFrames);
  const play = useMappingVizStore((s) => s.play);
  const pause = useMappingVizStore((s) => s.pause);
  const reset = useMappingVizStore((s) => s.reset);
  const setFrameIndex = useMappingVizStore((s) => s.setFrameIndex);

  const formDraft = useStaticSlamStore((s) => s.formDraft);
  const setFormDraft = useStaticSlamStore((s) => s.setFormDraft);
  const envelopes = useDataStore((s) => s.envelopes);

  const fileRef = useRef<HTMLInputElement>(null);
  const [urlText, setUrlText] = useState('');
  const [busy, setBusy] = useState(false);
  const [err, setErr] = useState<string | null>(null);

  const mappingPayload = useMemo(() => {
    const e = Object.values(envelopes).find((x) => x.schema === SCHEMAS.Mapping);
    return e?.payload ?? null;
  }, [envelopes]);

  const loadFromFiles = async (list: FileList | null) => {
    if (!list?.length) return;
    setBusy(true);
    setErr(null);
    try {
      const files = Array.from(list);
      const metaFile = files.find(isMetaFile);
      const images = sortMappingFrameFiles(files.filter(isImageFile));
      if (!images.length) throw new Error('请选择至少一张 PNG/PGM 等图像');

      let originX = formDraft.originX;
      let originY = formDraft.originY;
      let resolution = formDraft.resolution;
      let widthPx: number | undefined;
      let heightPx: number | undefined;

      if (metaFile) {
        const meta = parseStaticSlamSidecar(await metaFile.text(), 'auto');
        originX = meta.originX;
        originY = meta.originY;
        resolution = meta.resolution;
        widthPx = meta.widthPx;
        heightPx = meta.heightPx;
        setFormDraft({ originX, originY, resolution });
      }

      const urls = images.map((f) => URL.createObjectURL(f));
      const probeSrc = urls[0];
      if (widthPx == null || heightPx == null) {
        const size = await probeImageSize(probeSrc);
        widthPx = widthPx ?? size.w;
        heightPx = heightPx ?? size.h;
      }

      const meta: MappingBasemapMeta = {
        originX,
        originY,
        resolution,
        widthPx,
        heightPx,
      };
      loadDemoFrames(urls, meta);
      useStaticSlamStore.getState().setBasemap(
        basemapFromDemoFrame({
          imageSrc: urls[0],
          meta,
          label: images[0].name,
          source: 'file',
        }),
      );
      useLayerStore.getState().setLayer('basemap', true);
    } catch (e) {
      setErr(e instanceof Error ? e.message : String(e));
    } finally {
      setBusy(false);
      if (fileRef.current) fileRef.current.value = '';
    }
  };

  const loadFromUrlJson = async () => {
    setBusy(true);
    setErr(null);
    try {
      const parsed = JSON.parse(urlText) as unknown;
      if (!Array.isArray(parsed) || !parsed.every((x) => typeof x === 'string')) {
        throw new Error('请粘贴 URL 字符串数组 JSON，例如 ["http://…/1.png"]');
      }
      const urls = parsed as string[];
      if (!urls.length) throw new Error('URL 列表为空');
      const size = await probeImageSize(urls[0]);
      const meta: MappingBasemapMeta = {
        originX: formDraft.originX,
        originY: formDraft.originY,
        resolution: formDraft.resolution,
        widthPx: size.w,
        heightPx: size.h,
      };
      loadDemoFrames(urls, meta);
      useStaticSlamStore.getState().setBasemap(
        basemapFromDemoFrame({
          imageSrc: urls[0],
          meta,
          label: 'url-list',
          source: 'url',
        }),
      );
      useLayerStore.getState().setLayer('basemap', true);
    } catch (e) {
      setErr(e instanceof Error ? e.message : String(e));
    } finally {
      setBusy(false);
    }
  };

  const n = demo.frameUrls.length;

  return (
    <div className="panel mapping-panel">
      <h3>Mapping</h3>
      <p className="hint">
        Map 上 basemap / map 图层可在 Channels 或底图工具栏切换。
      </p>

      <div className="mapping-mode-row" role="group" aria-label="模式">
        <button
          type="button"
          className={mode === 'demo' ? 'active' : undefined}
          onClick={() => setMode('demo')}
        >
          演示回放
        </button>
        <button
          type="button"
          className={mode === 'live' ? 'active' : undefined}
          onClick={() => setMode('live')}
        >
          实时建图
        </button>
      </div>

      {mode === 'demo' ? (
        <div className="mapping-demo-block">
          <input
            ref={fileRef}
            type="file"
            accept=".png,.jpg,.jpeg,.bmp,.pgm,.webp,.yaml,.yml,.json,image/*"
            multiple
            hidden
            onChange={(e) => void loadFromFiles(e.target.files)}
          />
          <div className="row" style={{ gap: 8, flexWrap: 'wrap' }}>
            <button type="button" disabled={busy} onClick={() => fileRef.current?.click()}>
              选择帧…
            </button>
            <button
              type="button"
              disabled={!n || demo.playing}
              onClick={() => play()}
            >
              播放
            </button>
            <button type="button" disabled={!demo.playing} onClick={() => pause()}>
              暂停
            </button>
            <button type="button" disabled={!n} onClick={() => reset()}>
              重置
            </button>
            <button type="button" disabled={!n} onClick={() => clearDemoFrames()}>
              清除
            </button>
          </div>

          <div className="mapping-field-row">
            <label htmlFor="mapping-interval">间隔 ms</label>
            <input
              id="mapping-interval"
              type="number"
              min={50}
              max={10000}
              step={50}
              value={demo.intervalMs}
              onChange={(e) => setIntervalMs(Number(e.target.value) || 500)}
            />
          </div>

          <div className="mapping-field-row">
            <label>originX</label>
            <input
              type="number"
              step="any"
              value={formDraft.originX}
              onChange={(e) => setFormDraft({ originX: Number(e.target.value) })}
            />
            <label>originY</label>
            <input
              type="number"
              step="any"
              value={formDraft.originY}
              onChange={(e) => setFormDraft({ originY: Number(e.target.value) })}
            />
            <label>res</label>
            <input
              type="number"
              step="any"
              value={formDraft.resolution}
              onChange={(e) => setFormDraft({ resolution: Number(e.target.value) })}
            />
          </div>

          {n > 0 ? (
            <>
              <div className="mapping-progress">
                帧 {demo.frameIndex + 1}/{n}
                {demo.playing ? ' · 播放中' : ''}
              </div>
              <input
                type="range"
                min={0}
                max={Math.max(0, n - 1)}
                value={demo.frameIndex}
                onChange={(e) => setFrameIndex(Number(e.target.value))}
              />
            </>
          ) : (
            <p className="hint">多选建图阶段图（可附带 yaml/json sidecar）。</p>
          )}

          {demo.playing ? (
            <p className="hint">播放中：静态底图工具栏已禁用，请先暂停。</p>
          ) : null}

          <label className="hint" htmlFor="mapping-url-json">
            或粘贴 URL JSON 数组
          </label>
          <textarea
            id="mapping-url-json"
            className="mapping-url-json"
            rows={3}
            value={urlText}
            onChange={(e) => setUrlText(e.target.value)}
            placeholder='["/assets/stage_1.png","/assets/stage_2.png"]'
          />
          <button type="button" disabled={busy || !urlText.trim()} onClick={() => void loadFromUrlJson()}>
            加载 URL 列表
          </button>
        </div>
      ) : (
        <div className="mapping-live-block">
          <label className="mapping-check">
            <input
              type="checkbox"
              checked={mirror}
              onChange={(e) => setMirrorMapToBasemap(e.target.checked)}
            />
            mirrorMapToBasemap（把 map Occupancy 烘焙到底图）
          </label>
          {mirror ? (
            <p className="hint">开启后可关 Channels map 层，避免与 mirror 双重绘制。</p>
          ) : null}
          <pre className="mapping-json">
            {mappingPayload
              ? JSON.stringify(mappingPayload, null, 2)
              : 'Waiting for Mapping…'}
          </pre>
        </div>
      )}

      {err ? <p className="error">{err}</p> : null}
      {busy ? <p className="hint">加载中…</p> : null}
    </div>
  );
}
