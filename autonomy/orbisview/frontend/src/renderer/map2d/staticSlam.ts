import { decodePgm } from './pgmDecode';

export interface StaticSlamMeta {
  originX: number;
  originY: number;
  resolution: number;
  widthPx?: number;
  heightPx?: number;
  image?: string;
}

export interface StaticSlamBasemap {
  imageSrc: string;
  originX: number;
  originY: number;
  resolution: number;
  widthPx: number;
  heightPx: number;
  label?: string;
  source: 'file' | 'url' | 'asset';
}

function parseOriginList(raw: string): { x: number; y: number } | null {
  const m = raw.match(/\[\s*([-\d.eE+]+)\s*,\s*([-\d.eE+]+)/);
  if (!m) return null;
  return { x: Number(m[1]), y: Number(m[2]) };
}

export function parseStaticSlamSidecar(
  text: string,
  format: 'yaml' | 'json' | 'auto' = 'auto',
): StaticSlamMeta {
  const trimmed = text.trim();
  const asJson =
    format === 'json' ||
    (format === 'auto' && (trimmed.startsWith('{') || trimmed.startsWith('[')));
  if (asJson) {
    const j = JSON.parse(trimmed) as Record<string, unknown>;
    const origin = Array.isArray(j.origin) ? j.origin : null;
    const originX =
      typeof j.startX === 'number' ? j.startX : origin ? Number(origin[0]) : 0;
    const originY =
      typeof j.startY === 'number' ? j.startY : origin ? Number(origin[1]) : 0;
    return {
      originX,
      originY,
      resolution: Number(j.resolution ?? 0.05),
      widthPx: typeof j.xGridCount === 'number' ? j.xGridCount : undefined,
      heightPx: typeof j.yGridCount === 'number' ? j.yGridCount : undefined,
      image: typeof j.image === 'string' ? j.image : undefined,
    };
  }

  const lines = trimmed.split(/\r?\n/);
  let image: string | undefined;
  let resolution = 0.05;
  let originX = 0;
  let originY = 0;
  let widthPx: number | undefined;
  let heightPx: number | undefined;
  for (const line of lines) {
    const t = line.trim();
    if (!t || t.startsWith('#')) continue;
    const kv = t.match(/^([A-Za-z0-9_]+)\s*:\s*(.*)$/);
    if (!kv) continue;
    const key = kv[1];
    const val = kv[2].trim();
    if (key === 'image') image = val.replace(/^["']|["']$/g, '');
    else if (key === 'resolution') resolution = Number(val);
    else if (key === 'origin') {
      const o = parseOriginList(val);
      if (o) {
        originX = o.x;
        originY = o.y;
      }
    } else if (key === 'startX') originX = Number(val);
    else if (key === 'startY') originY = Number(val);
    else if (key === 'xGridCount') widthPx = Number(val);
    else if (key === 'yGridCount') heightPx = Number(val);
  }
  return { originX, originY, resolution, widthPx, heightPx, image };
}

export function staticSlamWorldSize(meta: {
  widthPx: number;
  heightPx: number;
  resolution: number;
}): { worldW: number; worldH: number } {
  return {
    worldW: meta.widthPx * meta.resolution,
    worldH: meta.heightPx * meta.resolution,
  };
}

export function staticSlamCorners(meta: {
  originX: number;
  originY: number;
  widthPx: number;
  heightPx: number;
  resolution: number;
}): { x0: number; y0: number; x1: number; y1: number } {
  const { worldW, worldH } = staticSlamWorldSize(meta);
  return {
    x0: meta.originX,
    y0: meta.originY,
    x1: meta.originX + worldW,
    y1: meta.originY + worldH,
  };
}

export interface StaticSlamCanvasHandle {
  canvas: HTMLCanvasElement;
  worldW: number;
  worldH: number;
  originX: number;
  originY: number;
}

function looksLikePgm(src: string, contentType?: string | null): boolean {
  if (contentType?.includes('x-portable-graymap') || contentType?.includes('pgm')) return true;
  return /\.pgm($|\?)/i.test(src);
}

export async function probeImageSize(src: string): Promise<{ w: number; h: number }> {
  if (looksLikePgm(src)) {
    const buf = await (await fetch(src)).arrayBuffer();
    const raster = decodePgm(buf);
    return { w: raster.width, h: raster.height };
  }
  return new Promise((resolve, reject) => {
    const img = new Image();
    img.crossOrigin = 'anonymous';
    img.onload = () => resolve({ w: img.naturalWidth, h: img.naturalHeight });
    img.onerror = () => reject(new Error(`Failed to load image: ${src}`));
    img.src = src;
  });
}

async function rasterToCanvas(
  raster: { data: Uint8ClampedArray; width: number; height: number },
  maxEdge: number,
): Promise<HTMLCanvasElement> {
  const scale = Math.min(1, maxEdge / Math.max(raster.width, raster.height));
  const tw = Math.max(1, Math.round(raster.width * scale));
  const th = Math.max(1, Math.round(raster.height * scale));
  const canvas = document.createElement('canvas');
  canvas.width = tw;
  canvas.height = th;
  const ctx = canvas.getContext('2d');
  if (!ctx) throw new Error('2d context unavailable');
  if (tw === raster.width && th === raster.height) {
    const img = ctx.createImageData(tw, th);
    img.data.set(raster.data);
    ctx.putImageData(img, 0, 0);
  } else {
    const full = document.createElement('canvas');
    full.width = raster.width;
    full.height = raster.height;
    const fctx = full.getContext('2d');
    if (!fctx) throw new Error('2d context unavailable');
    const img = fctx.createImageData(raster.width, raster.height);
    img.data.set(raster.data);
    fctx.putImageData(img, 0, 0);
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(full, 0, 0, tw, th);
  }
  return canvas;
}

export async function loadStaticSlamCanvas(
  basemap: StaticSlamBasemap,
  maxEdge = 1024,
): Promise<StaticSlamCanvasHandle> {
  const { worldW, worldH } = staticSlamWorldSize(basemap);
  let canvas: HTMLCanvasElement;

  if (looksLikePgm(basemap.imageSrc)) {
    const buf = await (await fetch(basemap.imageSrc)).arrayBuffer();
    canvas = await rasterToCanvas(decodePgm(buf), maxEdge);
  } else {
    const resp = await fetch(basemap.imageSrc).catch(() => null);
    const ct = resp?.headers.get('content-type');
    if (resp && looksLikePgm(basemap.imageSrc, ct)) {
      canvas = await rasterToCanvas(decodePgm(await resp.arrayBuffer()), maxEdge);
    } else if (resp && ct?.includes('octet-stream') && /\.pgm/i.test(basemap.imageSrc)) {
      canvas = await rasterToCanvas(decodePgm(await resp.arrayBuffer()), maxEdge);
    } else {
      const blobUrl = basemap.imageSrc;
      const img = await new Promise<HTMLImageElement>((resolve, reject) => {
        const el = new Image();
        el.crossOrigin = 'anonymous';
        el.onload = () => resolve(el);
        el.onerror = () => reject(new Error(`Failed to load image: ${blobUrl}`));
        el.src = blobUrl;
      });
      const scale = Math.min(1, maxEdge / Math.max(img.naturalWidth, img.naturalHeight));
      const tw = Math.max(1, Math.round(img.naturalWidth * scale));
      const th = Math.max(1, Math.round(img.naturalHeight * scale));
      canvas = document.createElement('canvas');
      canvas.width = tw;
      canvas.height = th;
      const ctx = canvas.getContext('2d');
      if (!ctx) throw new Error('2d context unavailable');
      ctx.imageSmoothingEnabled = false;
      ctx.drawImage(img, 0, 0, tw, th);
    }
  }

  return {
    canvas,
    worldW,
    worldH,
    originX: basemap.originX,
    originY: basemap.originY,
  };
}

export class StaticSlamCanvasCache {
  private key = '';
  private handle: StaticSlamCanvasHandle | null = null;
  private inflight: Promise<StaticSlamCanvasHandle> | null = null;

  async get(basemap: StaticSlamBasemap): Promise<StaticSlamCanvasHandle> {
    const next = [
      basemap.imageSrc,
      basemap.originX,
      basemap.originY,
      basemap.resolution,
      basemap.widthPx,
      basemap.heightPx,
    ].join('|');
    if (this.handle && this.key === next) return this.handle;
    if (this.inflight && this.key === next) return this.inflight;
    this.key = next;
    this.inflight = loadStaticSlamCanvas(basemap).then((h) => {
      this.handle = h;
      this.inflight = null;
      return h;
    });
    return this.inflight;
  }

  clear(): void {
    this.key = '';
    this.handle = null;
    this.inflight = null;
  }
}

export const sharedStaticSlamCanvasCache = new StaticSlamCanvasCache();

