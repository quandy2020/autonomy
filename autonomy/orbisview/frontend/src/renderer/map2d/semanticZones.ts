export interface SemanticZoneNorm {
  id: string;
  zoneType: string;
  polygon: [number, number][];
  fill: string;
  stroke: string;
  strokeWidth: number;
  label?: string;
}

export function zoneTypePalette(zoneType: string): { fill: string; stroke: string } {
  const t = zoneType.toLowerCase();
  if (t.includes('keepout') || t.includes('forbid') || t === 'no_go') {
    return { fill: 'rgba(239,83,80,0.35)', stroke: 'rgba(239,83,80,0.95)' };
  }
  if (t.includes('passable') || t.includes('free') || t === 'go') {
    return { fill: 'rgba(102,187,106,0.30)', stroke: 'rgba(76,175,80,0.95)' };
  }
  if (t.includes('room') || t.includes('area')) {
    return { fill: 'rgba(66,165,245,0.28)', stroke: 'rgba(33,150,243,0.95)' };
  }
  return { fill: 'rgba(158,158,158,0.28)', stroke: 'rgba(189,189,189,0.9)' };
}

export function colorRgbaToCss(
  c: { r?: number; g?: number; b?: number; a?: number } | undefined,
  opacity?: number,
): string | null {
  if (!c) return null;
  const r = Number(c.r);
  const g = Number(c.g);
  const b = Number(c.b);
  if (![r, g, b].every((v) => Number.isFinite(v))) return null;
  // Support 0–1 float or 0–255 int
  const to255 = (v: number) => (v <= 1 ? Math.round(v * 255) : Math.round(v));
  let a = Number.isFinite(Number(c.a)) ? Number(c.a) : 1;
  if (a > 1) a = a / 255;
  if (typeof opacity === 'number' && Number.isFinite(opacity)) {
    a *= Math.max(0, Math.min(1, opacity));
  }
  return `rgba(${to255(r)},${to255(g)},${to255(b)},${Math.max(0, Math.min(1, a))})`;
}

function asRecord(raw: unknown): Record<string, unknown> | null {
  return raw && typeof raw === 'object' && !Array.isArray(raw)
    ? (raw as Record<string, unknown>)
    : null;
}

export function normalizePolygon(raw: unknown): [number, number][] | null {
  if (!Array.isArray(raw)) return null;
  const out: [number, number][] = [];
  for (const p of raw) {
    if (Array.isArray(p) && p.length >= 2) {
      const x = Number(p[0]);
      const y = Number(p[1]);
      if (Number.isFinite(x) && Number.isFinite(y)) out.push([x, y]);
      continue;
    }
    const o = asRecord(p);
    if (o) {
      const x = Number(o.x);
      const y = Number(o.y);
      if (Number.isFinite(x) && Number.isFinite(y)) out.push([x, y]);
    }
  }
  return out.length >= 3 ? out : null;
}

export function normalizeSemanticZone(raw: unknown): SemanticZoneNorm | null {
  const o = asRecord(raw);
  if (!o) return null;
  const polygon = normalizePolygon(o.polygon);
  if (!polygon) return null;
  const zoneType = String(o.zone_type ?? o.zoneType ?? 'unknown');
  const palette = zoneTypePalette(zoneType);
  const fillOpacity =
    typeof o.fill_opacity === 'number'
      ? o.fill_opacity
      : typeof o.fillOpacity === 'number'
        ? o.fillOpacity
        : undefined;
  const fill =
    colorRgbaToCss(
      (o.fill_color ?? o.fillColor) as { r?: number; g?: number; b?: number; a?: number },
      fillOpacity,
    ) ?? palette.fill;
  const stroke =
    colorRgbaToCss(
      (o.outline_color ?? o.outlineColor) as {
        r?: number;
        g?: number;
        b?: number;
        a?: number;
      },
    ) ?? palette.stroke;
  const strokeWidth = Number(o.outline_width ?? o.outlineWidth ?? 1.5);
  const id = String(o.id ?? `zone-${polygon[0][0]}-${polygon[0][1]}`);
  const labelRaw = o.label;
  const label = typeof labelRaw === 'string' && labelRaw.trim() ? labelRaw.trim() : undefined;
  return {
    id,
    zoneType,
    polygon,
    fill,
    stroke,
    strokeWidth: Number.isFinite(strokeWidth) ? strokeWidth : 1.5,
    label,
  };
}

export function normalizeSemanticZoneArray(payload: unknown): SemanticZoneNorm[] {
  const o = asRecord(payload);
  const list = Array.isArray(payload)
    ? payload
    : Array.isArray(o?.zones)
      ? o!.zones
      : [];
  const out: SemanticZoneNorm[] = [];
  for (const z of list) {
    const n = normalizeSemanticZone(z);
    if (n) out.push(n);
  }
  return out;
}
