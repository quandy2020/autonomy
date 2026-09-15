/** Foxglove-style numeric message paths: `pose.x`, `speed[0]`, `wheels[1].rpm`. */

const SKIP_KEYS = new Set([
  'type',
  'data',
  'data_b64',
  'payload_b64',
  'bytes',
  'image',
  'rgb',
  'rgba',
  'points',
  'intensities',
  'ranges',
  'occupancy',
  'cells',
]);

const MAX_ARRAY_INDEX = 32;
const MAX_PATHS = 400;

export function collectNumericPaths(
  value: unknown,
  prefix = '',
  out: string[] = [],
  depth = 0,
): string[] {
  if (depth > 8 || out.length >= MAX_PATHS) return out;

  if (typeof value === 'number' && Number.isFinite(value)) {
    if (prefix) out.push(prefix);
    return out;
  }
  if (typeof value === 'boolean') {
    if (prefix) out.push(prefix);
    return out;
  }

  if (Array.isArray(value)) {
    // Homogeneous number[] → offer first few indices only (not every sample).
    if (value.length > 0 && typeof value[0] === 'number') {
      const n = Math.min(value.length, MAX_ARRAY_INDEX);
      for (let i = 0; i < n && out.length < MAX_PATHS; i++) {
        const p = prefix ? `${prefix}[${i}]` : `[${i}]`;
        out.push(p);
      }
      return out;
    }
    const n = Math.min(value.length, MAX_ARRAY_INDEX);
    for (let i = 0; i < n && out.length < MAX_PATHS; i++) {
      const p = prefix ? `${prefix}[${i}]` : `[${i}]`;
      collectNumericPaths(value[i], p, out, depth + 1);
    }
    return out;
  }

  if (value && typeof value === 'object') {
    for (const [k, v] of Object.entries(value as Record<string, unknown>)) {
      if (SKIP_KEYS.has(k)) continue;
      if (out.length >= MAX_PATHS) break;
      const p = prefix ? `${prefix}.${k}` : k;
      collectNumericPaths(v, p, out, depth + 1);
    }
  }
  return out;
}

function tokenizePath(path: string): string[] {
  return path.match(/[^.\[\]]+|\[\d+\]/g) ?? [];
}

export function resolvePath(root: unknown, path: string): unknown {
  let cur: unknown = root;
  for (const token of tokenizePath(path)) {
    if (cur == null) return undefined;
    if (token.startsWith('[')) {
      const i = Number(token.slice(1, -1));
      if (!Array.isArray(cur) || !Number.isFinite(i)) return undefined;
      cur = cur[i];
    } else {
      if (typeof cur !== 'object') return undefined;
      cur = (cur as Record<string, unknown>)[token];
    }
  }
  return cur;
}

export function envelopeJsonPayload(env: {
  payload?: unknown;
  payload_b64?: string;
  encoding?: string;
} | undefined): unknown {
  if (!env) return undefined;
  const raw = env.payload;
  if (raw && typeof raw === 'object') return raw;
  if (typeof raw === 'string' && raw.length > 0) {
    const t = raw.trimStart();
    if (t.startsWith('{') || t.startsWith('[')) {
      try {
        return JSON.parse(raw);
      } catch {
        return undefined;
      }
    }
  }
  return undefined;
}

export function readNumericPath(root: unknown, path: string): number | null {
  const tryPath = (p: string): number | null => {
    const v = resolvePath(root, p);
    if (typeof v === 'number' && Number.isFinite(v)) return v;
    if (typeof v === 'boolean') return v ? 1 : 0;
    return null;
  };

  const direct = tryPath(path);
  if (direct != null) return direct;

  // Descriptor paths vs reduced / alias JSON (TwistStamped, teleop helpers).
  const alts: string[] = [];
  if (path.startsWith('twist.')) alts.push(path.slice('twist.'.length));
  if (path.startsWith('linear.') || path.startsWith('angular.')) {
    alts.push(`twist.${path}`);
  }
  if (path === 'vx') alts.push('linear.x', 'twist.linear.x');
  if (path === 'wz') alts.push('angular.z', 'twist.angular.z');
  if (path === 'linear.x' || path === 'twist.linear.x') alts.push('vx');
  if (path === 'angular.z' || path === 'twist.angular.z') alts.push('wz');

  for (const a of alts) {
    const y = tryPath(a);
    if (y != null) return y;
  }
  return null;
}
