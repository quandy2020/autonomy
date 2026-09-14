import {
  getDisplayTypeDef,
  resolveDisplayType,
  type DisplayTypeDef,
} from '@/components/Channels/displayTypes';
import type { ChannelDisplay } from '@/store/displayStore';
import { SCHEMAS, type StreamEnvelope } from '@/store/websocket/types';
import type { LayerKey } from '@/store/layoutStore';

/** Map-relevant roles bound from Channels displays. */
export type MapDisplayRole =
  | 'pose'
  | 'path'
  | 'map'
  | 'costmap'
  | 'laser'
  | 'footprint'
  | 'tf'
  | 'obstacles'
  | 'prediction'
  | 'vectormap'
  | 'pointcloud'
  | 'twist'
  | 'chassis'
  | 'navigation'
  | 'range'
  | 'semantic'
  | 'floors';

const ROLE_LAYER: Partial<Record<MapDisplayRole, LayerKey>> = {
  pose: 'robot',
  path: 'path',
  map: 'map',
  costmap: 'costmap',
  laser: 'laser',
  footprint: 'footprint',
  tf: 'tf',
  obstacles: 'obstacles',
  prediction: 'prediction',
  vectormap: 'vectormap',
  pointcloud: 'pointcloud',
  range: 'laser',
  semantic: 'semantic',
};

const ROLE_TYPE_IDS: Record<MapDisplayRole, string[]> = {
  pose: [
    'geometry_msgs/Pose2D',
    'geometry_msgs/Pose',
    'geometry_msgs/PoseStamped',
    'geometry_msgs/Pose2DStamped',
    'nav_msgs/Odometry',
  ],
  path: ['nav_msgs/Path', 'nav_msgs/Route'],
  map: ['map_msgs/OccupancyGrid', 'nav_msgs/OccupancyGrid'],
  costmap: ['map_msgs/OccupancyGrid', 'nav_msgs/OccupancyGrid', 'nav_msgs/Costmap'],
  laser: ['sensor_msgs/LaserScan', 'sensor_msgs/MultiEchoLaserScan'],
  footprint: ['geometry_msgs/Polygon', 'geometry_msgs/PolygonStamped'],
  tf: ['tf2_msgs/TFMessage'],
  obstacles: [
    'visualization_msgs/Marker',
    'visualization_msgs/MarkerArray',
    'vision_msgs/Detection2DArray',
    'vision_msgs/Detection3DArray',
  ],
  prediction: ['visualization_msgs/MarkerArray'],
  vectormap: ['map_msgs/OccupancyGrid'],
  pointcloud: ['sensor_msgs/PointCloud2', 'sensor_msgs/PointCloud'],
  twist: ['geometry_msgs/Twist', 'geometry_msgs/TwistStamped'],
  chassis: ['vehicle_msgs/RobotState', 'status_msgs/Status'],
  navigation: ['nav_msgs/Goals', 'nav_msgs/Path'],
  range: ['sensor_msgs/Range'],
  semantic: ['strata_msgs/SemanticZoneArray'],
  floors: ['strata_msgs/FloorInfoArray'],
};

const ROLE_FALLBACK_SCHEMAS: Record<MapDisplayRole, string[]> = {
  pose: [SCHEMAS.Pose2D],
  path: [SCHEMAS.Path2D, SCHEMAS.RoutePath],
  map: [SCHEMAS.OccupancyGrid],
  costmap: [SCHEMAS.OccupancyGrid],
  laser: [SCHEMAS.LaserScan],
  footprint: [SCHEMAS.RobotFootprint],
  tf: [SCHEMAS.TfTree],
  obstacles: [SCHEMAS.ObstacleArray],
  prediction: [SCHEMAS.PredictionObstacles],
  vectormap: [SCHEMAS.VectorMap],
  pointcloud: [SCHEMAS.PointCloud2],
  twist: [SCHEMAS.Twist2D],
  chassis: [SCHEMAS.ChassisState],
  navigation: [SCHEMAS.Navigation],
  range: [],
  semantic: [SCHEMAS.SemanticZoneArray],
  floors: [SCHEMAS.FloorInfoArray],
};

function defForDisplay(d: ChannelDisplay): DisplayTypeDef {
  return (
    getDisplayTypeDef(d.typeId) ??
    resolveDisplayType(
      d.typeId.split('/')[0] ?? 'std_msgs',
      d.typeId.split('/')[1] ?? d.typeId,
    )
  );
}

function isCostmapChannel(name: string): boolean {
  return /costmap/i.test(name);
}

function displayMatchesRole(d: ChannelDisplay, role: MapDisplayRole): boolean {
  const typeOk = ROLE_TYPE_IDS[role].includes(d.typeId);
  if (!typeOk) {
    // Also accept inferred defs whose matchSchemas hit role fallback schemas.
    const def = defForDisplay(d);
    const schemas = ROLE_FALLBACK_SCHEMAS[role];
    if (!def.matchSchemas.some((s) => schemas.includes(s) || schemas.some((f) => s.includes(f)))) {
      return false;
    }
  }

  if (role === 'map') return !isCostmapChannel(d.channel || String(d.props.name ?? ''));
  if (role === 'costmap') return isCostmapChannel(d.channel || String(d.props.name ?? d.typeId));
  if (role === 'vectormap') return /vector|lane|hdmap/i.test(d.channel || d.typeId);
  if (role === 'prediction') return /prediction/i.test(d.channel || d.typeId);
  return true;
}

function configuredForRole(displays: ChannelDisplay[], role: MapDisplayRole): boolean {
  return displays.some((d) => d.channel && displayMatchesRole(d, role));
}

/** Enabled displays for a role that currently have an envelope. */
export function pickDisplayEnvelopes(
  envelopes: Record<string, StreamEnvelope>,
  displays: ChannelDisplay[],
  role: MapDisplayRole,
): StreamEnvelope[] {
  const configured = configuredForRole(displays, role);
  if (configured) {
    return displays
      .filter((d) => d.enabled && d.channel && displayMatchesRole(d, role))
      .map((d) => envelopes[d.channel])
      .filter((e): e is StreamEnvelope => !!e);
  }

  // Legacy: no Channels display for this role → schema scan.
  const schemas = ROLE_FALLBACK_SCHEMAS[role];
  const all = Object.values(envelopes).filter((e) => schemas.includes(e.schema));
  if (role === 'map') {
    return all.filter((e) => !isCostmapChannel(e.channel)).slice(0, 1);
  }
  if (role === 'costmap') {
    return all.filter((e) => isCostmapChannel(e.channel)).slice(0, 1);
  }
  return all.slice(0, 1);
}

export function pickDisplayEnvelope(
  envelopes: Record<string, StreamEnvelope>,
  displays: ChannelDisplay[],
  role: MapDisplayRole,
): StreamEnvelope | undefined {
  return pickDisplayEnvelopes(envelopes, displays, role)[0];
}

export function asPayload<T>(env: { payload?: unknown } | undefined): T | null {
  if (!env?.payload || typeof env.payload !== 'object') return null;
  return env.payload as T;
}

/**
 * Layer visibility:
 * - If Channels has any display for a role → that role follows enabled displays.
 * - Otherwise keep the Layers toggle (legacy / unconfigured types).
 */
export function effectiveMapLayers(
  layers: Record<LayerKey, boolean>,
  displays: ChannelDisplay[],
): Record<LayerKey, boolean> {
  const out = { ...layers };
  (Object.keys(ROLE_LAYER) as MapDisplayRole[]).forEach((role) => {
    const key = ROLE_LAYER[role];
    if (!key) return;
    if (!configuredForRole(displays, role)) return;
    out[key] = displays.some((d) => d.enabled && d.channel && displayMatchesRole(d, role));
  });
  return out;
}
