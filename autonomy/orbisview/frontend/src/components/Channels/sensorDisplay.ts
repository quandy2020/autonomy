import {
  getDisplayTypeDef,
  resolveDisplayType,
  type PropValue,
} from '@/components/Channels/displayTypes';
import type { ChannelDisplay } from '@/store/displayStore';
import type { StreamEnvelope } from '@/store/websocket/types';
import { pickDisplayEnvelopes, asPayload } from '@/components/Channels/mapDisplayBinding';

export interface LaserScanPayload {
  angle_min: number;
  angle_increment: number;
  ranges: number[];
  range_min?: number;
  range_max?: number;
  /** LaserScan header.frame_id (e.g. laser_link). */
  frame_id?: string;
}

export interface CloudPoint {
  x: number;
  y: number;
  z: number;
  i?: number;
}

export interface RangePayload {
  range: number;
  field_of_view?: number;
  min_range?: number;
  max_range?: number;
  radiation_type?: number | string;
}

export interface SensorStyle {
  color: string;
  size: number;
  alpha: number;
  colorMode: 'intensity' | 'height' | 'rgb';
}

const DEFAULT_LASER: SensorStyle = {
  color: '#ffcc80',
  size: 0.05,
  alpha: 1,
  colorMode: 'intensity',
};

const DEFAULT_CLOUD: SensorStyle = {
  color: '#4fc3f7',
  size: 0.06,
  alpha: 0.9,
  colorMode: 'intensity',
};

const DEFAULT_RANGE: SensorStyle = {
  color: '#80cbc4',
  size: 0.08,
  alpha: 0.7,
  colorMode: 'intensity',
};

function num(v: PropValue | undefined, fallback: number): number {
  return typeof v === 'number' && Number.isFinite(v) ? v : fallback;
}

function str(v: PropValue | undefined, fallback: string): string {
  return typeof v === 'string' && v ? v : fallback;
}

function styleFromDisplay(d: ChannelDisplay | undefined, fallback: SensorStyle): SensorStyle {
  if (!d) return fallback;
  const mode = str(d.props.colorMode, fallback.colorMode);
  return {
    color: str(d.props.color, fallback.color),
    size: num(d.props.size, fallback.size),
    alpha: num(d.props.alpha, fallback.alpha),
    colorMode:
      mode === 'height' || mode === 'rgb' || mode === 'intensity' ? mode : fallback.colorMode,
  };
}

function firstEnabled(
  displays: ChannelDisplay[],
  typeIds: string[],
): ChannelDisplay | undefined {
  return displays.find((d) => d.enabled && d.channel && typeIds.includes(d.typeId));
}

/** Enabled LaserScan overlays (channel + style), in display order. */
export function resolveLaserOverlays(
  envelopes: Record<string, StreamEnvelope>,
  displays: ChannelDisplay[],
): { scan: LaserScanPayload; style: SensorStyle; channel: string }[] {
  const envs = pickDisplayEnvelopes(envelopes, displays, 'laser');
  if (!envs.length) return [];

  const laserDisplays = displays.filter(
    (d) => d.enabled && d.channel && d.typeId === 'sensor_msgs/LaserScan',
  );

  return envs
    .map((env) => {
      const d =
        laserDisplays.find((x) => x.channel === env.channel) ??
        firstEnabled(displays, ['sensor_msgs/LaserScan']);
      const scan = asPayload<LaserScanPayload>(env);
      if (!scan?.ranges?.length) return null;
      return {
        scan: {
          ...scan,
          frame_id: scan.frame_id || env.frame_id || undefined,
        },
        style: styleFromDisplay(d, DEFAULT_LASER),
        channel: env.channel,
      };
    })
    .filter((x): x is NonNullable<typeof x> => !!x);
}

export function resolveCloudOverlay(
  envelopes: Record<string, StreamEnvelope>,
  displays: ChannelDisplay[],
): { points: CloudPoint[]; style: SensorStyle; channel: string } | null {
  const env = pickDisplayEnvelopes(envelopes, displays, 'pointcloud')[0];
  if (!env) return null;
  const d =
    displays.find(
      (x) =>
        x.enabled &&
        x.channel === env.channel &&
        (x.typeId === 'sensor_msgs/PointCloud2' || x.typeId === 'sensor_msgs/PointCloud'),
    ) ?? firstEnabled(displays, ['sensor_msgs/PointCloud2', 'sensor_msgs/PointCloud']);
  const payload = asPayload<{ points?: CloudPoint[] }>(env);
  const points = payload?.points;
  if (!points?.length) return null;
  return {
    points,
    style: styleFromDisplay(d, DEFAULT_CLOUD),
    channel: env.channel,
  };
}

export function resolveRangeOverlays(
  envelopes: Record<string, StreamEnvelope>,
  displays: ChannelDisplay[],
): { range: RangePayload; style: SensorStyle; channel: string }[] {
  const typeIds = ['sensor_msgs/Range'];
  return displays
    .filter((d) => d.enabled && d.channel && typeIds.includes(d.typeId))
    .map((d) => {
      const env = envelopes[d.channel];
      const range = asPayload<RangePayload>(env);
      if (!range || !Number.isFinite(range.range)) return null;
      return { range, style: styleFromDisplay(d, DEFAULT_RANGE), channel: d.channel };
    })
    .filter((x): x is NonNullable<typeof x> => !!x);
}

/** Image / depth / compressed channels selected via Displays. */
export function enabledImageChannels(displays: ChannelDisplay[]): string[] {
  const ids = new Set([
    'sensor_msgs/Image',
    'sensor_msgs/CompressedImage',
    'sensor_msgs/CameraInfo',
  ]);
  return displays
    .filter((d) => d.enabled && d.channel && ids.has(d.typeId))
    .map((d) => d.channel);
}

export function resolvePathStyle(
  displays: ChannelDisplay[],
): { color: string; lineWidth: number; alpha: number } {
  const d =
    displays.find(
      (x) =>
        x.enabled &&
        x.channel &&
        (x.typeId === 'nav_msgs/Path' ||
          x.typeId === 'nav_msgs/Route' ||
          x.typeId === 'nav_msgs/PathStamped'),
    ) ??
    displays.find(
      (x) =>
        x.typeId === 'nav_msgs/Path' ||
        x.typeId === 'nav_msgs/Route' ||
        x.typeId === 'nav_msgs/PathStamped',
    );
  return {
    color: str(d?.props.color, '#4fc3f7'),
    lineWidth: num(d?.props.lineWidth, 2),
    alpha: num(d?.props.alpha, 0.9),
  };
}

export function sensorDisplayLabel(d: ChannelDisplay): string {
  const def =
    getDisplayTypeDef(d.typeId) ??
    resolveDisplayType(d.typeId.split('/')[0] ?? '', d.typeId.split('/')[1] ?? d.typeId);
  return String(d.props.name || def.label);
}

export function hexToRgba(hex: string, alpha: number): string {
  const h = hex.replace('#', '');
  const full = h.length === 3 ? h.split('').map((c) => c + c).join('') : h;
  const n = Number.parseInt(full, 16);
  if (!Number.isFinite(n)) return `rgba(255,204,128,${alpha})`;
  const r = (n >> 16) & 255;
  const g = (n >> 8) & 255;
  const b = n & 255;
  return `rgba(${r},${g},${b},${Math.max(0, Math.min(1, alpha))})`;
}

export function hexToThree(hex: string): number {
  const h = hex.replace('#', '');
  const full = h.length === 3 ? h.split('').map((c) => c + c).join('') : h;
  const n = Number.parseInt(full, 16);
  return Number.isFinite(n) ? n : 0xffcc80;
}
