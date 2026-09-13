import { useEffect, useRef } from 'react';
import { useDataStore } from '@/store/dataStore';
import { SCHEMAS } from '@/store/websocket/types';
import { useLayerStore } from '@/store/layoutStore';

function depthToRgb(v: number): [number, number, number] {
  // turbo-ish: near (255) -> warm, far (0) -> cool
  const t = Math.max(0, Math.min(1, v / 255));
  const r = Math.round(255 * Math.min(1, t * 2));
  const g = Math.round(255 * (1 - Math.abs(t - 0.5) * 2));
  const b = Math.round(255 * Math.min(1, (1 - t) * 2));
  return [r, g, b];
}

function drawMono(
  canvas: HTMLCanvasElement,
  width: number,
  height: number,
  data: number[],
  mode: 'gray' | 'depth',
) {
  const ctx = canvas.getContext('2d');
  if (!ctx) return;
  canvas.width = width;
  canvas.height = height;
  const img = ctx.createImageData(width, height);
  for (let i = 0; i < width * height; i++) {
    const v = data[i] ?? 0;
    if (mode === 'depth') {
      const [r, g, b] = depthToRgb(v);
      img.data[i * 4] = r;
      img.data[i * 4 + 1] = g;
      img.data[i * 4 + 2] = b;
    } else {
      img.data[i * 4] = v;
      img.data[i * 4 + 1] = v;
      img.data[i * 4 + 2] = v;
    }
    img.data[i * 4 + 3] = 255;
  }
  ctx.putImageData(img, 0, 0);
}

export function ImagePanel() {
  const imageRef = useRef<HTMLCanvasElement>(null);
  const depthRef = useRef<HTMLCanvasElement>(null);
  const envelopes = useDataStore((s) => s.envelopes);
  const layers = useLayerStore();

  useEffect(() => {
    const env = Object.values(envelopes).find((e) => e.schema === SCHEMAS.Image);
    const payload = env?.payload as
      | { width: number; height: number; data: number[] }
      | undefined;
    if (imageRef.current && payload && layers.image) {
      drawMono(imageRef.current, payload.width, payload.height, payload.data, 'gray');
    }
  }, [envelopes, layers.image]);

  useEffect(() => {
    const env = Object.values(envelopes).find((e) => e.schema === SCHEMAS.DepthImage);
    const payload = env?.payload as
      | { width: number; height: number; data: number[] }
      | undefined;
    if (depthRef.current && payload && layers.depth) {
      drawMono(depthRef.current, payload.width, payload.height, payload.data, 'depth');
    }
  }, [envelopes, layers.depth]);

  const imageStale = Object.values(envelopes).find((e) => e.schema === SCHEMAS.Image)?.stale;
  const depthStale = Object.values(envelopes).find((e) => e.schema === SCHEMAS.DepthImage)?.stale;

  return (
    <div className="panel">
      <div className="row" style={{ gap: 16, flexWrap: 'wrap' }}>
        {layers.image ? (
          <div>
            <h4>Image {imageStale ? <span className="stale-badge">stale</span> : null}</h4>
            <canvas
              ref={imageRef}
              className="image-canvas"
              style={{ width: 320, height: 240, imageRendering: 'pixelated' }}
            />
          </div>
        ) : null}
        {layers.depth ? (
          <div>
            <h4>Depth {depthStale ? <span className="stale-badge">stale</span> : null}</h4>
            <canvas
              ref={depthRef}
              className="image-canvas"
              style={{ width: 320, height: 240, imageRendering: 'pixelated' }}
            />
          </div>
        ) : null}
      </div>
    </div>
  );
}
