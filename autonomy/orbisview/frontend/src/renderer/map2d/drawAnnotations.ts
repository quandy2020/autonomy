import type { AnnotationDraft } from '@/store/annotationStore';
import {
  poiKindColor,
  type MapDrawShape,
  type MapPoi,
} from './annotations';

export function drawAnnotations(
  ctx: CanvasRenderingContext2D,
  args: {
    pois: MapPoi[];
    shapes: MapDrawShape[];
    draft: AnnotationDraft | null;
    draftPreview: { x: number; y: number } | null;
    selectedId: string | null;
    showPoi: boolean;
    showDraw: boolean;
    toScreen: (x: number, y: number) => readonly [number, number] | [number, number];
    scale: number;
  },
): void {
  const {
    pois,
    shapes,
    draft,
    draftPreview,
    selectedId,
    showPoi,
    showDraw,
    toScreen,
    scale,
  } = args;

  if (showDraw) {
    for (const s of shapes) {
      strokeShape(ctx, s.points, toScreen, {
        close: s.kind === 'polygon',
        fill: s.fill ?? (s.kind === 'polygon' ? 'rgba(255,167,38,0.22)' : undefined),
        stroke: s.stroke ?? '#ffa726',
        lineWidth: s.id === selectedId ? 2.5 : 1.5,
      });
      if (s.label && scale >= 10) {
        labelAtCentroid(ctx, s.points, toScreen, s.label);
      }
    }
    if (draft?.points.length) {
      const pts = [...draft.points];
      if (draftPreview) pts.push([draftPreview.x, draftPreview.y]);
      ctx.save();
      ctx.setLineDash([6, 4]);
      strokeShape(ctx, pts, toScreen, {
        close: draft.kind === 'polygon' && pts.length >= 3,
        fill:
          draft.kind === 'polygon' && pts.length >= 3
            ? 'rgba(129,212,250,0.18)'
            : undefined,
        stroke: '#81d4fa',
        lineWidth: 1.5,
      });
      ctx.restore();
      for (const [x, y] of draft.points) {
        const [sx, sy] = toScreen(x, y);
        ctx.fillStyle = '#81d4fa';
        ctx.beginPath();
        ctx.arc(sx, sy, 3.5, 0, Math.PI * 2);
        ctx.fill();
      }
    }
  }

  if (showPoi) {
    for (const p of pois) {
      const [sx, sy] = toScreen(p.x, p.y);
      const color = p.color ?? poiKindColor(p.kind);
      const selected = p.id === selectedId;
      ctx.beginPath();
      ctx.fillStyle = color;
      ctx.arc(sx, sy, selected ? 7 : 5.5, 0, Math.PI * 2);
      ctx.fill();
      ctx.strokeStyle = selected ? '#fff' : 'rgba(0,0,0,0.55)';
      ctx.lineWidth = selected ? 2 : 1;
      ctx.stroke();
      if (p.label && scale >= 8) {
        ctx.fillStyle = 'rgba(236,239,241,0.92)';
        ctx.font = '11px system-ui, sans-serif';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'bottom';
        ctx.fillText(p.label, sx, sy - 8);
      }
    }
  }
}

function strokeShape(
  ctx: CanvasRenderingContext2D,
  points: [number, number][],
  toScreen: (x: number, y: number) => readonly [number, number] | [number, number],
  opts: { close: boolean; fill?: string; stroke: string; lineWidth: number },
) {
  if (points.length < 2) return;
  ctx.beginPath();
  points.forEach(([x, y], i) => {
    const [sx, sy] = toScreen(x, y);
    if (i === 0) ctx.moveTo(sx, sy);
    else ctx.lineTo(sx, sy);
  });
  if (opts.close) ctx.closePath();
  if (opts.fill) {
    ctx.fillStyle = opts.fill;
    ctx.fill();
  }
  ctx.strokeStyle = opts.stroke;
  ctx.lineWidth = opts.lineWidth;
  ctx.stroke();
}

function labelAtCentroid(
  ctx: CanvasRenderingContext2D,
  points: [number, number][],
  toScreen: (x: number, y: number) => readonly [number, number] | [number, number],
  label: string,
) {
  let cx = 0;
  let cy = 0;
  for (const [x, y] of points) {
    cx += x;
    cy += y;
  }
  cx /= points.length;
  cy /= points.length;
  const [sx, sy] = toScreen(cx, cy);
  ctx.fillStyle = 'rgba(236,239,241,0.9)';
  ctx.font = '11px system-ui, sans-serif';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'middle';
  ctx.fillText(label, sx, sy);
}
