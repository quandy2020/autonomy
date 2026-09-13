/** Shared frontend helpers (Dreamview src/utils counterpart). */
export function clamp(v: number, lo: number, hi: number): number {
  return Math.max(lo, Math.min(hi, v));
}
