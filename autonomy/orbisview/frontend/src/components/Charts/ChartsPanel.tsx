import { useEffect, useMemo, useRef, useState } from 'react';
import { useDataStore } from '@/store/dataStore';
import { SCHEMAS } from '@/store/websocket/types';

function drawSeries(canvas: HTMLCanvasElement, series: number[][], colors: string[]) {
  const ctx = canvas.getContext('2d');
  if (!ctx) return;
  const w = canvas.width;
  const h = canvas.height;
  ctx.fillStyle = '#0a0e12';
  ctx.fillRect(0, 0, w, h);
  series.forEach((values, si) => {
    if (values.length < 2) return;
    const min = Math.min(...values);
    const max = Math.max(...values);
    const span = Math.max(1e-6, max - min);
    ctx.strokeStyle = colors[si] ?? '#4fc3f7';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    values.forEach((v, i) => {
      const x = (i / (values.length - 1)) * (w - 2) + 1;
      const y = h - 2 - ((v - min) / span) * (h - 4);
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.stroke();
  });
}

export function ChartsPanel() {
  const envelopes = useDataStore((s) => s.envelopes);
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [history, setHistory] = useState<{ vx: number; wz: number }[]>([]);

  const chassis = useMemo(() => {
    const e = Object.values(envelopes).find((x) => x.schema === SCHEMAS.ChassisState);
    return e?.payload as { vx?: number; wz?: number } | undefined;
  }, [envelopes]);

  const planning = useMemo(() => {
    const e = Object.values(envelopes).find((x) => x.schema === SCHEMAS.PlanningDebug);
    return e?.payload as
      | {
          speed?: number[];
          acceleration?: number[];
          heading_error?: number[];
          station_error?: number[];
        }
      | undefined;
  }, [envelopes]);

  useEffect(() => {
    if (chassis?.vx == null && chassis?.wz == null) return;
    setHistory((prev) =>
      [...prev, { vx: chassis?.vx ?? 0, wz: chassis?.wz ?? 0 }].slice(-80),
    );
  }, [chassis?.vx, chassis?.wz]);

  useEffect(() => {
    if (!canvasRef.current) return;
    drawSeries(
      canvasRef.current,
      [history.map((h) => h.vx), history.map((h) => h.wz)],
      ['#69f0ae', '#ffd54f'],
    );
  }, [history]);

  return (
    <div className="panel">
      <h3 style={{ marginTop: 0 }}>Charts</h3>
      <p className="hint">green=vx · yellow=wz (chassis)</p>
      <canvas ref={canvasRef} width={420} height={120} className="spark" />
      {planning ? (
        <div style={{ marginTop: 12 }}>
          <p className="hint">PlanningDebug sparklines</p>
          <MiniSpark label="speed" values={planning.speed ?? []} color="#4fc3f7" />
          <MiniSpark label="accel" values={planning.acceleration ?? []} color="#ce93d8" />
          <MiniSpark label="heading_err" values={planning.heading_error ?? []} color="#ef9a9a" />
          <MiniSpark label="station_err" values={planning.station_error ?? []} color="#ffcc80" />
        </div>
      ) : null}
    </div>
  );
}

function MiniSpark({
  label,
  values,
  color,
}: {
  label: string;
  values: number[];
  color: string;
}) {
  const ref = useRef<HTMLCanvasElement>(null);
  useEffect(() => {
    if (ref.current) drawSeries(ref.current, [values], [color]);
  }, [values, color]);
  return (
    <div className="row" style={{ gap: 8, margin: '0.35rem 0' }}>
      <span style={{ width: 90, fontSize: 12 }} className="muted">
        {label}
      </span>
      <canvas ref={ref} width={280} height={36} className="spark" />
    </div>
  );
}
