import { useRef, useState } from 'react';
import { STATIC_SLAM_ASSETS } from '@/config/parameters';
import {
  parseStaticSlamSidecar,
  probeImageSize,
  type StaticSlamBasemap,
} from '@/renderer/map2d/staticSlam';
import { useStaticSlamStore } from '@/store/staticSlamStore';
import { useLayerStore } from '@/store/layoutStore';
import { useMapViewStore } from '@/store/mapViewStore';

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

async function applyBasemap(b: StaticSlamBasemap) {
  useStaticSlamStore.getState().setBasemap(b);
  useLayerStore.getState().setLayer('basemap', true);
  const w = b.widthPx * b.resolution;
  const h = b.heightPx * b.resolution;
  useMapViewStore
    .getState()
    .setStatusMsg(
      `底图 ${w.toFixed(1)}×${h.toFixed(1)} m · res ${b.resolution}` +
        (b.label ? ` · ${b.label}` : ''),
    );
}

/** Popover: load static SLAM basemap from files / URL / assets. */
export function StaticSlamLoadPop({ onClose }: { onClose: () => void }) {
  const fileRef = useRef<HTMLInputElement>(null);
  const formDraft = useStaticSlamStore((s) => s.formDraft);
  const setFormDraft = useStaticSlamStore((s) => s.setFormDraft);
  const basemap = useStaticSlamStore((s) => s.basemap);
  const clearBasemap = useStaticSlamStore((s) => s.clearBasemap);
  const basemapOn = useLayerStore((s) => s.basemap);
  const setLayer = useLayerStore((s) => s.setLayer);
  const [url, setUrl] = useState('');
  const [busy, setBusy] = useState(false);
  const [err, setErr] = useState<string | null>(null);

  const onFiles = async (list: FileList | null) => {
    if (!list?.length) return;
    setBusy(true);
    setErr(null);
    try {
      const files = Array.from(list);
      const metaFile = files.find(isMetaFile);
      const imgFile = files.find(isImageFile);
      if (!imgFile) throw new Error('请选择 PNG/PGM/BMP 等图像文件');

      let originX = formDraft.originX;
      let originY = formDraft.originY;
      let resolution = formDraft.resolution;
      let widthPx: number | undefined;
      let heightPx: number | undefined;

      if (metaFile) {
        const text = await metaFile.text();
        const meta = parseStaticSlamSidecar(text, 'auto');
        originX = meta.originX;
        originY = meta.originY;
        resolution = meta.resolution;
        widthPx = meta.widthPx;
        heightPx = meta.heightPx;
        setFormDraft({ originX, originY, resolution });
      }

      const imageSrc = URL.createObjectURL(imgFile);
      if (widthPx == null || heightPx == null) {
        const size = await probeImageSize(imageSrc);
        widthPx = widthPx ?? size.w;
        heightPx = heightPx ?? size.h;
      }

      await applyBasemap({
        imageSrc,
        originX,
        originY,
        resolution,
        widthPx,
        heightPx,
        label: imgFile.name,
        source: 'file',
      });
      onClose();
    } catch (e) {
      setErr(e instanceof Error ? e.message : String(e));
    } finally {
      setBusy(false);
    }
  };

  const onUrl = async () => {
    const imageUrl = url.trim();
    if (!imageUrl) return;
    setBusy(true);
    setErr(null);
    try {
      let originX = formDraft.originX;
      let originY = formDraft.originY;
      let resolution = formDraft.resolution;
      let widthPx: number | undefined;
      let heightPx: number | undefined;

      const metaUrl = imageUrl.replace(/\.(png|jpg|jpeg|bmp|pgm|webp)(\?.*)?$/i, '.yaml');
      if (metaUrl !== imageUrl) {
        try {
          const res = await fetch(metaUrl);
          if (res.ok) {
            const meta = parseStaticSlamSidecar(await res.text(), 'auto');
            originX = meta.originX;
            originY = meta.originY;
            resolution = meta.resolution;
            widthPx = meta.widthPx;
            heightPx = meta.heightPx;
            setFormDraft({ originX, originY, resolution });
          }
        } catch {
          /* optional sidecar */
        }
      }

      if (widthPx == null || heightPx == null) {
        const size = await probeImageSize(imageUrl);
        widthPx = widthPx ?? size.w;
        heightPx = heightPx ?? size.h;
      }

      await applyBasemap({
        imageSrc: imageUrl,
        originX,
        originY,
        resolution,
        widthPx,
        heightPx,
        label: imageUrl.split('/').pop(),
        source: 'url',
      });
      onClose();
    } catch (e) {
      setErr(e instanceof Error ? e.message : String(e));
    } finally {
      setBusy(false);
    }
  };

  const onAsset = async (asset: (typeof STATIC_SLAM_ASSETS)[number]) => {
    setBusy(true);
    setErr(null);
    try {
      let originX = formDraft.originX;
      let originY = formDraft.originY;
      let resolution = formDraft.resolution;
      let widthPx: number | undefined;
      let heightPx: number | undefined;
      if (asset.metaUrl) {
        const res = await fetch(asset.metaUrl);
        if (res.ok) {
          const meta = parseStaticSlamSidecar(await res.text(), 'auto');
          originX = meta.originX;
          originY = meta.originY;
          resolution = meta.resolution;
          widthPx = meta.widthPx;
          heightPx = meta.heightPx;
          setFormDraft({ originX, originY, resolution });
        }
      }
      if (widthPx == null || heightPx == null) {
        const size = await probeImageSize(asset.imageUrl);
        widthPx = widthPx ?? size.w;
        heightPx = heightPx ?? size.h;
      }
      await applyBasemap({
        imageSrc: asset.imageUrl,
        originX,
        originY,
        resolution,
        widthPx,
        heightPx,
        label: asset.label,
        source: 'asset',
      });
      onClose();
    } catch (e) {
      setErr(e instanceof Error ? e.message : String(e));
    } finally {
      setBusy(false);
    }
  };

  return (
    <div className="map-basemap-pop" role="dialog" aria-label="加载底图">
      <div className="map-basemap-pop-head">
        <strong>静态底图</strong>
        <button type="button" className="link" onClick={onClose}>
          关闭
        </button>
      </div>

      <label className="map-basemap-row">
        <span>origin X</span>
        <input
          type="number"
          step="any"
          value={formDraft.originX}
          onChange={(e) => setFormDraft({ originX: Number(e.target.value) })}
        />
      </label>
      <label className="map-basemap-row">
        <span>origin Y</span>
        <input
          type="number"
          step="any"
          value={formDraft.originY}
          onChange={(e) => setFormDraft({ originY: Number(e.target.value) })}
        />
      </label>
      <label className="map-basemap-row">
        <span>resolution</span>
        <input
          type="number"
          step="any"
          value={formDraft.resolution}
          onChange={(e) => setFormDraft({ resolution: Number(e.target.value) })}
        />
      </label>

      <input
        ref={fileRef}
        type="file"
        accept="image/*,.pgm,.yaml,.yml,.json"
        multiple
        hidden
        onChange={(e) => void onFiles(e.target.files)}
      />
      <button
        type="button"
        className="map-basemap-btn"
        disabled={busy}
        onClick={() => fileRef.current?.click()}
      >
        选择文件（图 ± yaml）
      </button>

      <div className="map-basemap-url">
        <input
          type="url"
          placeholder="https://…/map.png"
          value={url}
          onChange={(e) => setUrl(e.target.value)}
        />
        <button type="button" className="map-basemap-btn" disabled={busy || !url.trim()} onClick={() => void onUrl()}>
          URL
        </button>
      </div>

      {STATIC_SLAM_ASSETS.length ? (
        <div className="map-basemap-assets">
          {STATIC_SLAM_ASSETS.map((a) => (
            <button
              key={a.label}
              type="button"
              className="map-basemap-btn"
              disabled={busy}
              onClick={() => void onAsset(a)}
            >
              {a.label}
            </button>
          ))}
        </div>
      ) : null}

      <label className="map-basemap-check">
        <input
          type="checkbox"
          checked={basemapOn}
          onChange={(e) => setLayer('basemap', e.target.checked)}
        />
        显示底图图层
      </label>

      {basemap ? (
        <button
          type="button"
          className="map-basemap-btn danger"
          onClick={() => {
            clearBasemap();
            useMapViewStore.getState().setStatusMsg('已清除静态底图');
          }}
        >
          清除底图
        </button>
      ) : null}

      {err ? <p className="map-basemap-err">{err}</p> : null}
      {busy ? <p className="hint">加载中…</p> : null}
    </div>
  );
}
