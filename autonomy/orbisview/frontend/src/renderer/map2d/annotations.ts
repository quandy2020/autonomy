export type PoiKind = 'charger' | 'elevator' | 'custom';
export type DrawShapeKind = 'polygon' | 'polyline';

export interface MapPoi {
  id: string;
  x: number;
  y: number;
  yaw?: number;
  kind: PoiKind;
  label?: string;
  color?: string;
}

export interface MapDrawShape {
  id: string;
  kind: DrawShapeKind;
  points: [number, number][];
  label?: string;
  stroke?: string;
  fill?: string;
}

export const MAX_POIS = 500;
export const MAX_SHAPE_POINTS = 200;

export function newAnnotationId(): string {
  if (typeof crypto !== 'undefined' && typeof crypto.randomUUID === 'function') {
    return crypto.randomUUID();
  }
  return `ann-${Date.now().toString(36)}-${Math.random().toString(36).slice(2, 9)}`;
}

export function poiKindColor(kind: PoiKind): string {
  if (kind === 'charger') return '#66bb6a';
  if (kind === 'elevator') return '#42a5f5';
  return '#ffca28';
}

function asRecord(raw: unknown): Record<string, unknown> | null {
  return raw && typeof raw === 'object' && !Array.isArray(raw)
    ? (raw as Record<string, unknown>)
    : null;
}

function normalizePoint(raw: unknown): [number, number] | null {
  if (Array.isArray(raw) && raw.length >= 2) {
    const x = Number(raw[0]);
    const y = Number(raw[1]);
    return Number.isFinite(x) && Number.isFinite(y) ? [x, y] : null;
  }
  const o = asRecord(raw);
  if (!o) return null;
  const x = Number(o.x);
  const y = Number(o.y);
  return Number.isFinite(x) && Number.isFinite(y) ? [x, y] : null;
}

function normalizeKind(raw: unknown): PoiKind {
  if (raw === 'charger' || raw === 'elevator' || raw === 'custom') return raw;
  return 'custom';
}

function normalizeShapeKind(raw: unknown): DrawShapeKind | null {
  if (raw === 'polygon' || raw === 'polyline') return raw;
  return null;
}

export function normalizePoi(raw: unknown): MapPoi | null {
  const o = asRecord(raw);
  if (!o) return null;
  const x = Number(o.x);
  const y = Number(o.y);
  if (!Number.isFinite(x) || !Number.isFinite(y)) return null;
  const id = typeof o.id === 'string' && o.id ? o.id : newAnnotationId();
  const yaw = typeof o.yaw === 'number' && Number.isFinite(o.yaw) ? o.yaw : undefined;
  const label = typeof o.label === 'string' && o.label.trim() ? o.label.trim() : undefined;
  const color = typeof o.color === 'string' ? o.color : undefined;
  return { id, x, y, yaw, kind: normalizeKind(o.kind), label, color };
}

export function normalizeShape(raw: unknown): MapDrawShape | null {
  const o = asRecord(raw);
  if (!o) return null;
  const kind = normalizeShapeKind(o.kind);
  if (!kind) return null;
  if (!Array.isArray(o.points)) return null;
  const points: [number, number][] = [];
  for (const p of o.points.slice(0, MAX_SHAPE_POINTS)) {
    const n = normalizePoint(p);
    if (n) points.push(n);
  }
  const min = kind === 'polygon' ? 3 : 2;
  if (points.length < min) return null;
  const id = typeof o.id === 'string' && o.id ? o.id : newAnnotationId();
  const label = typeof o.label === 'string' && o.label.trim() ? o.label.trim() : undefined;
  const stroke = typeof o.stroke === 'string' ? o.stroke : undefined;
  const fill = typeof o.fill === 'string' ? o.fill : undefined;
  return { id, kind, points, label, stroke, fill };
}

export function parseAnnotationFixture(text: string): {
  pois: MapPoi[];
  shapes: MapDrawShape[];
} {
  const json = JSON.parse(text) as unknown;
  const o = asRecord(json);
  if (!o) throw new Error('fixture must be an object');
  const pois: MapPoi[] = [];
  if (Array.isArray(o.pois)) {
    for (const p of o.pois) {
      const n = normalizePoi(p);
      if (n) pois.push(n);
      if (pois.length >= MAX_POIS) break;
    }
  }
  const shapes: MapDrawShape[] = [];
  if (Array.isArray(o.shapes)) {
    for (const s of o.shapes) {
      const n = normalizeShape(s);
      if (n) shapes.push(n);
    }
  }
  return { pois, shapes };
}

export function serializeAnnotations(pois: MapPoi[], shapes: MapDrawShape[]): string {
  return JSON.stringify({ version: 1, pois, shapes }, null, 2);
}

export function hitTestPoi(
  pois: MapPoi[],
  x: number,
  y: number,
  radiusM: number,
): MapPoi | null {
  const r2 = radiusM * radiusM;
  let best: MapPoi | null = null;
  let bestD = Infinity;
  for (const p of pois) {
    const dx = p.x - x;
    const dy = p.y - y;
    const d = dx * dx + dy * dy;
    if (d <= r2 && d < bestD) {
      best = p;
      bestD = d;
    }
  }
  return best;
}
