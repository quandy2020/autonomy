import { useEffect, useMemo, useRef, useState } from 'react';
import { useDataStore } from '@/store/dataStore';
import { SCHEMAS } from '@/store/websocket/types';

type Sample = { t: number; dist: number; cover: number };

const MAX_SAMPLES = 120;

function drawSpark(
  canvas: HTMLCanvasElement,
  values: number[],
  color: string,
  yMin: number,
  yMax: number,
) {
  const ctx = canvas.getContext('2d');
  if (!ctx) return;
  const w = canvas.width;
  const h = canvas.height;
  ctx.fillStyle = '#0a0e12';
  ctx.fillRect(0, 0, w, h);
  ctx.strokeStyle = '#243040';
  ctx.beginPath();
  ctx.moveTo(0, h / 2);
  ctx.lineTo(w, h / 2);
  ctx.stroke();
  if (values.length < 2) return;
  const span = Math.max(1e-6, yMax - yMin);
  ctx.strokeStyle = color;
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  values.forEach((v, i) => {
    const x = (i / (values.length - 1)) * (w - 2) + 1;
    const y = h - 2 - ((v - yMin) / span) * (h - 4);
    if (i === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  });
  ctx.stroke();
}

export function PncMonitorPanel() {
  const envelopes = useDataStore((s) => s.envelopes);
  const [samples, setSamples] = useState<Sample[]>([]);
  const distRef = useRef<HTMLCanvasElement>(null);
  const coverRef = useRef<HTMLCanvasElement>(null);
  const speedRef = useRef<HTMLCanvasElement>(null);

  const nav = useMemo(() => {
    const e = Object.values(envelopes).find((x) => x.schema === SCHEMAS.Navigation);
    return e?.payload as
      | { state?: string; distance_remaining?: number; has_goal?: boolean; goal?: { x: number; y: number } }
      | undefined;
  }, [envelopes]);

  const explore = useMemo(() => {
    const e = Object.values(envelopes).find((x) => x.schema === SCHEMAS.Exploration);
    return e?.payload as
      | { frontier_count?: number; covered_ratio?: number }
      | undefined;
  }, [envelopes]);

  const planning = useMemo(() => {
    const e = Object.values(envelopes).find((x) => x.schema === SCHEMAS.PlanningDebug);
    return e?.payload as { speed?: number[]; heading_error?: number[] } | undefined;
  }, [envelopes]);

  useEffect(() => {
    const dist = nav?.distance_remaining;
    const cover = explore?.covered_ratio;
    if (dist == null && cover == null) return;
    setSamples((prev) => {
      const next = [
        ...prev,
        {
          t: Date.now(),
          dist: dist ?? prev[prev.length - 1]?.dist ?? 0,
          cover: cover ?? prev[prev.length - 1]?.cover ?? 0,
        },
      ];
      return next.slice(-MAX_SAMPLES);
    });
  }, [nav?.distance_remaining, explore?.covered_ratio]);

  useEffect(() => {
    if (distRef.current) {
      drawSpark(
        distRef.current,
        samples.map((s) => s.dist),
        '#4fc3f7',
        0,
        Math.max(2, ...samples.map((s) => s.dist), 1),
      );
    }
    if (coverRef.current) {
      drawSpark(
        coverRef.current,
        samples.map((s) => s.cover),
        '#69f0ae',
        0,
        1,
      );
    }
    if (speedRef.current && planning?.speed?.length) {
      drawSpark(speedRef.current, planning.speed, '#ce93d8', 0, Math.max(...planning.speed, 1));
    }
  }, [samples, planning?.speed]);

  return (
    <div className="panel">
      <h3>PNC Monitor</h3>
      <div className="row" style={{ gap: 16, flexWrap: 'wrap' }}>
        <div>
          <div className="muted">
            distance_remaining · {nav?.state ?? '—'} ·{' '}
            {nav?.distance_remaining?.toFixed(2) ?? '—'}
          </div>
          <canvas ref={distRef} width={320} height={80} className="spark" />
        </div>
        <div>
          <div className="muted">
            covered_ratio · frontiers={explore?.frontier_count ?? '—'} ·{' '}
            {explore?.covered_ratio != null
              ? `${(explore.covered_ratio * 100).toFixed(1)}%`
              : '—'}
          </div>
          <canvas ref={coverRef} width={320} height={80} className="spark" />
        </div>
        <div>
          <div className="muted">planning speed profile</div>
          <canvas ref={speedRef} width={320} height={80} className="spark" />
        </div>
      </div>
      <pre style={{ marginTop: 12 }}>
        {JSON.stringify(
          {
            navigation: nav ?? null,
            exploration: explore ?? null,
            heading_error: planning?.heading_error ?? null,
          },
          null,
          2,
        )}
      </pre>
    </div>
  );
}
