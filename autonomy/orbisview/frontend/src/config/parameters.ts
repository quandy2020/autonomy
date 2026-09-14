/** Frontend tunables (WS URL, ground-robot defaults). */

/** Prefer page hostname so LAN clients connect to the same machine as the UI. */
export function defaultWsUrl(port = 8766): string {
  if (typeof window === 'undefined') return `ws://127.0.0.1:${port}/ws`;
  const host = window.location.hostname || '127.0.0.1';
  const proto = window.location.protocol === 'https:' ? 'wss' : 'ws';
  return `${proto}://${host}:${port}/ws`;
}

/** @deprecated use defaultWsUrl() — kept for static imports / tests */
export const DEFAULT_WS_URL = 'ws://127.0.0.1:8766/ws';

export const DEFAULT_FOOTPRINT = {
  shape: 'RECT',
  length: 0.9,
  width: 0.56,
} as const;

/** Optional packaged SLAM assets (image + meta URLs). Empty = hide shortcuts. */
export const STATIC_SLAM_ASSETS: {
  label: string;
  imageUrl: string;
  metaUrl?: string;
}[] = [];
