import type { StaticSlamBasemap } from './staticSlam';

export interface FloorInfoNorm {
  id: string;
  name: string;
  level: number;
  slamImagePath?: string;
  originX: number;
  originY: number;
  widthPx: number;
  heightPx: number;
  resolution: number;
}

function asRecord(raw: unknown): Record<string, unknown> | null {
  return raw && typeof raw === 'object' && !Array.isArray(raw)
    ? (raw as Record<string, unknown>)
    : null;
}

export function normalizeFloorInfo(raw: unknown): FloorInfoNorm | null {
  const o = asRecord(raw);
  if (!o) return null;
  const id = String(o.id ?? '');
  if (!id) return null;
  const name = String(o.name ?? id);
  const level = Number(o.level ?? 0);
  const path = String(o.slam_image_path ?? o.slamImagePath ?? '').trim();
  const originX = Number(o.start_x ?? o.startX ?? o.originX ?? 0);
  const originY = Number(o.start_y ?? o.startY ?? o.originY ?? 0);
  const widthPx = Number(o.x_grid_count ?? o.xGridCount ?? o.widthPx ?? 0);
  const heightPx = Number(o.y_grid_count ?? o.yGridCount ?? o.heightPx ?? 0);
  const resolution = Number(o.resolution ?? 0.05);
  if (!(resolution > 0) || widthPx <= 0 || heightPx <= 0) {
    // Allow floors without slam grid (list-only)
    return {
      id,
      name,
      level: Number.isFinite(level) ? level : 0,
      slamImagePath: path || undefined,
      originX: Number.isFinite(originX) ? originX : 0,
      originY: Number.isFinite(originY) ? originY : 0,
      widthPx: widthPx > 0 ? widthPx : 1,
      heightPx: heightPx > 0 ? heightPx : 1,
      resolution: resolution > 0 ? resolution : 0.05,
    };
  }
  return {
    id,
    name,
    level: Number.isFinite(level) ? level : 0,
    slamImagePath: path || undefined,
    originX: Number.isFinite(originX) ? originX : 0,
    originY: Number.isFinite(originY) ? originY : 0,
    widthPx,
    heightPx,
    resolution,
  };
}

export function sortFloors(floors: FloorInfoNorm[]): FloorInfoNorm[] {
  return [...floors].sort((a, b) => {
    if (a.level !== b.level) return a.level - b.level;
    return a.name.localeCompare(b.name, undefined, { numeric: true });
  });
}

export function normalizeFloorInfoArray(payload: unknown): {
  floors: FloorInfoNorm[];
  activeFloorId: string | null;
} {
  const o = asRecord(payload);
  const list = Array.isArray(payload)
    ? payload
    : Array.isArray(o?.floors)
      ? o!.floors
      : [];
  const floors = sortFloors(
    list.map(normalizeFloorInfo).filter((f): f is FloorInfoNorm => !!f),
  );
  let activeFloorId =
    typeof o?.active_floor_id === 'string'
      ? o.active_floor_id
      : typeof o?.activeFloorId === 'string'
        ? o.activeFloorId
        : null;
  if (activeFloorId && !floors.some((f) => f.id === activeFloorId)) {
    activeFloorId = null;
  }
  if (!activeFloorId && floors.length) activeFloorId = floors[0].id;
  return { floors, activeFloorId };
}

/** http(s), blob, absolute /path, or non-empty relative (no file:) */
export function floorHasFetchableSlam(f: FloorInfoNorm): boolean {
  const p = f.slamImagePath?.trim();
  if (!p) return false;
  if (p.startsWith('file:')) return false;
  if (/^https?:\/\//i.test(p) || p.startsWith('blob:') || p.startsWith('/')) return true;
  // relative path like assets/foo.png
  return !p.includes('://');
}

export function resolveSlamImageSrc(path: string): string {
  const p = path.trim();
  if (/^https?:\/\//i.test(p) || p.startsWith('blob:') || p.startsWith('/')) return p;
  if (typeof window !== 'undefined' && window.location?.origin) {
    return new URL(p, window.location.origin + '/').href;
  }
  return p;
}

export function basemapFromFloor(f: FloorInfoNorm): StaticSlamBasemap | null {
  if (!floorHasFetchableSlam(f) || !f.slamImagePath) return null;
  if (!(f.resolution > 0) || f.widthPx <= 0 || f.heightPx <= 0) return null;
  return {
    imageSrc: resolveSlamImageSrc(f.slamImagePath),
    originX: f.originX,
    originY: f.originY,
    resolution: f.resolution,
    widthPx: f.widthPx,
    heightPx: f.heightPx,
    label: f.name,
    source: /^https?:\/\//i.test(f.slamImagePath) ? 'url' : 'asset',
  };
}
