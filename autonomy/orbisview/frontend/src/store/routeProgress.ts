/**
 * Route progress helpers: drop reached / in-obstacle waypoints for multi-nav.
 */

import type { OccupancyGridJson } from '@/renderer/map2d/types';
import type { Waypoint } from '@/store/waypointStore';

/** Sample OccupancyGrid at map frame (x,y). null if outside / invalid. */
export function sampleOccupancy(
  grid: OccupancyGridJson | null | undefined,
  x: number,
  y: number,
): number | null {
  if (!grid || !grid.width || !grid.height || !grid.resolution) return null;
  const ox = grid.origin?.x ?? 0;
  const oy = grid.origin?.y ?? 0;
  const col = Math.floor((x - ox) / grid.resolution);
  const row = Math.floor((y - oy) / grid.resolution);
  if (col < 0 || row < 0 || col >= grid.width || row >= grid.height) return null;
  const idx = row * grid.width + col;
  const v = grid.data?.[idx];
  return typeof v === 'number' ? v : null;
}

/** Occupancy / costmap cell treated as blocked for waypoint placement. */
export function isOccupancyBlocked(value: number | null): boolean {
  if (value == null) return false;
  // -1 unknown: keep (planner may still find a path).
  if (value < 0) return false;
  // Occupancy 0–100: ≥90 ≈ occupied. Costmap-as-occupancy uses same scale.
  return value >= 90;
}

/** Drop waypoints whose cells are lethal / near-occupied. Preserves order. */
export function filterFreeWaypoints(
  waypoints: Waypoint[],
  grid: OccupancyGridJson | null | undefined,
): { kept: Waypoint[]; removed: number } {
  if (!grid || !waypoints.length) {
    return { kept: waypoints, removed: 0 };
  }
  const kept: Waypoint[] = [];
  let removed = 0;
  for (const wp of waypoints) {
    if (isOccupancyBlocked(sampleOccupancy(grid, wp.x, wp.y))) {
      removed += 1;
      continue;
    }
    kept.push(wp);
  }
  return { kept, removed };
}

/** How many leading waypoints are within radius of (x,y). */
export function countReachedPrefix(
  waypoints: Waypoint[],
  x: number,
  y: number,
  radius: number,
): number {
  const r2 = radius * radius;
  let n = 0;
  for (const wp of waypoints) {
    const dx = wp.x - x;
    const dy = wp.y - y;
    if (dx * dx + dy * dy > r2) break;
    n += 1;
  }
  return n;
}
