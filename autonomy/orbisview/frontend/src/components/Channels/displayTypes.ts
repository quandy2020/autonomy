import catalogJson from './msgCatalog.generated.json';
import { SCHEMAS } from '@/store/websocket/types';

export interface MsgCatalogEntry {
  package: string;
  message: string;
  file: string;
}

export type PropValue = string | number | boolean;

export interface DisplayPropDef {
  key: string;
  label: string;
  kind: 'string' | 'number' | 'boolean' | 'select' | 'color' | 'topic';
  group: string;
  defaultValue: PropValue;
  options?: string[];
  min?: number;
  max?: number;
  step?: number;
}

export interface DisplayTypeDef {
  /** e.g. sensor_msgs/LaserScan */
  typeId: string;
  package: string;
  message: string;
  label: string;
  /** Compatible orbisview / wire schemas. */
  matchSchemas: string[];
  props: DisplayPropDef[];
}

const COMMON_PROPS: DisplayPropDef[] = [
  { key: 'name', label: 'Name', kind: 'string', group: 'Status', defaultValue: '' },
  { key: 'enabled', label: 'Enabled', kind: 'boolean', group: 'Status', defaultValue: true },
  { key: 'topic', label: 'Topic', kind: 'topic', group: 'Topic', defaultValue: '' },
  { key: 'maxHz', label: 'Max Hz', kind: 'number', group: 'Topic', defaultValue: 20, min: 1, max: 100, step: 1 },
];

function withCommon(extra: DisplayPropDef[]): DisplayPropDef[] {
  return [...COMMON_PROPS, ...extra];
}

/** Display types that OrbisView can visualize, keyed by automsgs type. */
export const DISPLAY_TYPE_DEFS: DisplayTypeDef[] = [
  {
    typeId: 'geometry_msgs/Pose2D',
    package: 'geometry_msgs',
    message: 'Pose2D',
    label: 'Pose 2D',
    matchSchemas: [SCHEMAS.Pose2D, 'geometry_msgs/Pose2D', 'Pose2D'],
    props: withCommon([
      { key: 'shape', label: 'Shape', kind: 'select', group: 'Style', defaultValue: 'arrow', options: ['arrow', 'axes', 'ring'] },
      { key: 'color', label: 'Color', kind: 'color', group: 'Style', defaultValue: '#69f0ae' },
      { key: 'shaftLength', label: 'Shaft Length', kind: 'number', group: 'Style', defaultValue: 0.5, min: 0.05, max: 5, step: 0.05 },
    ]),
  },
  {
    typeId: 'geometry_msgs/Twist',
    package: 'geometry_msgs',
    message: 'Twist',
    label: 'Twist',
    matchSchemas: [SCHEMAS.Twist2D, 'geometry_msgs/Twist', 'Twist2D'],
    props: withCommon([
      { key: 'showLinear', label: 'Show Linear', kind: 'boolean', group: 'Style', defaultValue: true },
      { key: 'showAngular', label: 'Show Angular', kind: 'boolean', group: 'Style', defaultValue: true },
    ]),
  },
  {
    typeId: 'nav_msgs/Path',
    package: 'nav_msgs',
    message: 'Path',
    label: 'Path',
    matchSchemas: [SCHEMAS.Path2D, 'nav_msgs/Path', 'Path2D'],
    props: withCommon([
      { key: 'lineWidth', label: 'Line Width', kind: 'number', group: 'Style', defaultValue: 2, min: 1, max: 12, step: 1 },
      { key: 'color', label: 'Color', kind: 'color', group: 'Style', defaultValue: '#4fc3f7' },
      { key: 'alpha', label: 'Alpha', kind: 'number', group: 'Style', defaultValue: 0.9, min: 0, max: 1, step: 0.05 },
    ]),
  },
  {
    typeId: 'nav_msgs/Odometry',
    package: 'nav_msgs',
    message: 'Odometry',
    label: 'Odometry',
    matchSchemas: ['nav_msgs/Odometry', 'Odometry'],
    props: withCommon([
      { key: 'keep', label: 'Keep', kind: 'number', group: 'History', defaultValue: 100, min: 1, max: 5000, step: 1 },
      { key: 'color', label: 'Color', kind: 'color', group: 'Style', defaultValue: '#81c784' },
    ]),
  },
  {
    typeId: 'map_msgs/OccupancyGrid',
    package: 'map_msgs',
    message: 'OccupancyGrid',
    label: 'Occupancy Grid',
    matchSchemas: [SCHEMAS.OccupancyGrid, 'map_msgs/OccupancyGrid', 'nav_msgs/OccupancyGrid', 'OccupancyGrid'],
    props: withCommon([
      { key: 'alpha', label: 'Alpha', kind: 'number', group: 'Style', defaultValue: 0.7, min: 0, max: 1, step: 0.05 },
      { key: 'colorScheme', label: 'Color Scheme', kind: 'select', group: 'Style', defaultValue: 'map', options: ['map', 'costmap', 'raw'] },
      { key: 'drawBehind', label: 'Draw Behind', kind: 'boolean', group: 'Style', defaultValue: true },
    ]),
  },
  {
    typeId: 'sensor_msgs/LaserScan',
    package: 'sensor_msgs',
    message: 'LaserScan',
    label: 'LaserScan',
    matchSchemas: [SCHEMAS.LaserScan, 'sensor_msgs/LaserScan', 'LaserScan'],
    props: withCommon([
      { key: 'size', label: 'Size (m)', kind: 'number', group: 'Style', defaultValue: 0.05, min: 0.01, max: 0.5, step: 0.01 },
      { key: 'alpha', label: 'Alpha', kind: 'number', group: 'Style', defaultValue: 1, min: 0, max: 1, step: 0.05 },
      { key: 'color', label: 'Color', kind: 'color', group: 'Style', defaultValue: '#ffcc80' },
    ]),
  },
  {
    typeId: 'sensor_msgs/MultiEchoLaserScan',
    package: 'sensor_msgs',
    message: 'MultiEchoLaserScan',
    label: 'MultiEcho LaserScan',
    matchSchemas: [SCHEMAS.LaserScan, 'sensor_msgs/MultiEchoLaserScan', 'MultiEchoLaserScan'],
    props: withCommon([
      { key: 'size', label: 'Size (m)', kind: 'number', group: 'Style', defaultValue: 0.05, min: 0.01, max: 0.5, step: 0.01 },
      { key: 'alpha', label: 'Alpha', kind: 'number', group: 'Style', defaultValue: 1, min: 0, max: 1, step: 0.05 },
      { key: 'color', label: 'Color', kind: 'color', group: 'Style', defaultValue: '#ffe082' },
    ]),
  },
  {
    typeId: 'sensor_msgs/Range',
    package: 'sensor_msgs',
    message: 'Range',
    label: 'Range',
    matchSchemas: ['sensor_msgs/Range', 'Range'],
    props: withCommon([
      { key: 'alpha', label: 'Alpha', kind: 'number', group: 'Style', defaultValue: 0.7, min: 0, max: 1, step: 0.05 },
      { key: 'color', label: 'Color', kind: 'color', group: 'Style', defaultValue: '#80cbc4' },
    ]),
  },
  {
    typeId: 'sensor_msgs/PointCloud2',
    package: 'sensor_msgs',
    message: 'PointCloud2',
    label: 'PointCloud2',
    matchSchemas: [SCHEMAS.PointCloud2, 'sensor_msgs/PointCloud2', 'PointCloud2'],
    props: withCommon([
      { key: 'size', label: 'Size', kind: 'number', group: 'Style', defaultValue: 0.06, min: 0.01, max: 0.5, step: 0.01 },
      { key: 'alpha', label: 'Alpha', kind: 'number', group: 'Style', defaultValue: 0.9, min: 0, max: 1, step: 0.05 },
      { key: 'style', label: 'Style', kind: 'select', group: 'Style', defaultValue: 'points', options: ['points', 'squares', 'flat_squares'] },
      { key: 'colorMode', label: 'Color Mode', kind: 'select', group: 'Style', defaultValue: 'intensity', options: ['intensity', 'height', 'rgb'] },
    ]),
  },
  {
    typeId: 'sensor_msgs/PointCloud',
    package: 'sensor_msgs',
    message: 'PointCloud',
    label: 'PointCloud',
    matchSchemas: [SCHEMAS.PointCloud2, 'sensor_msgs/PointCloud', 'PointCloud'],
    props: withCommon([
      { key: 'size', label: 'Size', kind: 'number', group: 'Style', defaultValue: 0.06, min: 0.01, max: 0.5, step: 0.01 },
      { key: 'alpha', label: 'Alpha', kind: 'number', group: 'Style', defaultValue: 0.9, min: 0, max: 1, step: 0.05 },
      { key: 'colorMode', label: 'Color Mode', kind: 'select', group: 'Style', defaultValue: 'intensity', options: ['intensity', 'height'] },
    ]),
  },
  {
    typeId: 'sensor_msgs/Image',
    package: 'sensor_msgs',
    message: 'Image',
    label: 'Image',
    matchSchemas: [SCHEMAS.Image, 'sensor_msgs/Image', 'Image'],
    props: withCommon([
      { key: 'transport', label: 'Transport', kind: 'select', group: 'Image', defaultValue: 'raw', options: ['raw', 'compressed'] },
      { key: 'queueSize', label: 'Queue Size', kind: 'number', group: 'Image', defaultValue: 1, min: 1, max: 10, step: 1 },
    ]),
  },
  {
    typeId: 'sensor_msgs/CompressedImage',
    package: 'sensor_msgs',
    message: 'CompressedImage',
    label: 'Compressed Image',
    matchSchemas: [SCHEMAS.Image, 'sensor_msgs/CompressedImage', 'CompressedImage'],
    props: withCommon([
      { key: 'queueSize', label: 'Queue Size', kind: 'number', group: 'Image', defaultValue: 1, min: 1, max: 10, step: 1 },
    ]),
  },
  {
    typeId: 'sensor_msgs/CameraInfo',
    package: 'sensor_msgs',
    message: 'CameraInfo',
    label: 'Camera Info',
    matchSchemas: ['sensor_msgs/CameraInfo', 'CameraInfo', SCHEMAS.DepthImage],
    props: withCommon([]),
  },
  {
    typeId: 'tf2_msgs/TFMessage',
    package: 'tf2_msgs',
    message: 'TFMessage',
    label: 'TF',
    matchSchemas: [SCHEMAS.TfTree, 'tf2_msgs/TFMessage', 'TfTree'],
    props: withCommon([
      { key: 'showNames', label: 'Show Names', kind: 'boolean', group: 'Frames', defaultValue: true },
      { key: 'showAxes', label: 'Show Axes', kind: 'boolean', group: 'Frames', defaultValue: true },
      { key: 'frameTimeout', label: 'Frame Timeout', kind: 'number', group: 'Frames', defaultValue: 15, min: 0, max: 60, step: 1 },
    ]),
  },
  {
    typeId: 'visualization_msgs/Marker',
    package: 'visualization_msgs',
    message: 'Marker',
    label: 'Marker',
    matchSchemas: ['visualization_msgs/Marker', 'Marker', SCHEMAS.ObstacleArray],
    props: withCommon([
      { key: 'namespaces', label: 'Namespaces', kind: 'string', group: 'Filter', defaultValue: '' },
      { key: 'alpha', label: 'Alpha', kind: 'number', group: 'Style', defaultValue: 1, min: 0, max: 1, step: 0.05 },
    ]),
  },
  {
    typeId: 'visualization_msgs/MarkerArray',
    package: 'visualization_msgs',
    message: 'MarkerArray',
    label: 'Marker Array',
    matchSchemas: ['visualization_msgs/MarkerArray', 'MarkerArray', SCHEMAS.ObstacleArray, SCHEMAS.PredictionObstacles],
    props: withCommon([
      { key: 'namespaces', label: 'Namespaces', kind: 'string', group: 'Filter', defaultValue: '' },
    ]),
  },
  {
    typeId: 'nav_msgs/OccupancyGrid',
    package: 'nav_msgs',
    message: 'OccupancyGrid',
    label: 'Map (nav_msgs)',
    matchSchemas: [SCHEMAS.OccupancyGrid, 'nav_msgs/OccupancyGrid', 'OccupancyGrid'],
    props: withCommon([
      { key: 'alpha', label: 'Alpha', kind: 'number', group: 'Style', defaultValue: 0.7, min: 0, max: 1, step: 0.05 },
      { key: 'colorScheme', label: 'Color Scheme', kind: 'select', group: 'Style', defaultValue: 'map', options: ['map', 'costmap', 'raw'] },
    ]),
  },
  {
    typeId: 'geometry_msgs/Polygon',
    package: 'geometry_msgs',
    message: 'Polygon',
    label: 'Polygon / Footprint',
    matchSchemas: [SCHEMAS.RobotFootprint, 'geometry_msgs/Polygon', 'RobotFootprint'],
    props: withCommon([
      { key: 'color', label: 'Color', kind: 'color', group: 'Style', defaultValue: '#80cbc4' },
      { key: 'lineWidth', label: 'Line Width', kind: 'number', group: 'Style', defaultValue: 1.5, min: 0.5, max: 8, step: 0.5 },
    ]),
  },
];

export const MSG_CATALOG = catalogJson as MsgCatalogEntry[];

export const MSG_PACKAGES = [
  ...new Set([
    ...MSG_CATALOG.map((m) => m.package),
    ...DISPLAY_TYPE_DEFS.map((d) => d.package),
  ]),
].sort((a, b) => a.localeCompare(b));

const DEF_BY_TYPE = new Map(DISPLAY_TYPE_DEFS.map((d) => [d.typeId, d]));

export function getDisplayTypeDef(typeId: string): DisplayTypeDef | undefined {
  return DEF_BY_TYPE.get(typeId);
}

/** Prefer curated display defs; fall back to generic topic props for any catalog msg. */
export function resolveDisplayType(packageName: string, message: string): DisplayTypeDef {
  const typeId = `${packageName}/${message}`;
  const known = DEF_BY_TYPE.get(typeId);
  if (known) return known;
  return {
    typeId,
    package: packageName,
    message,
    label: message,
    matchSchemas: [typeId, message, `${packageName}.${message}`],
    props: withCommon([]),
  };
}

export function channelMatchesType(
  channel: { schema: string; msg_type?: string; name: string },
  def: DisplayTypeDef,
): boolean {
  const hay = `${channel.schema} ${channel.msg_type ?? ''} ${channel.name}`.toLowerCase();
  return def.matchSchemas.some((s) => hay.includes(s.toLowerCase()));
}

/** Infer automsgs package/message from a live ChannelInfo. */
export function inferTypeFromChannel(channel: {
  schema: string;
  msg_type?: string;
  name: string;
}): { package: string; message: string } {
  for (const def of DISPLAY_TYPE_DEFS) {
    if (channelMatchesType(channel, def)) {
      return { package: def.package, message: def.message };
    }
  }

  const raw = (channel.msg_type || channel.schema || '').trim();
  const dotted = raw.match(/(?:^|[.])([a-z][a-z0-9_]*)\.([A-Z][A-Za-z0-9_]*)$/);
  if (dotted) return { package: dotted[1], message: dotted[2] };

  const slash = raw.match(/^([a-z][a-z0-9_]*)\/([A-Za-z0-9_]+)$/);
  if (slash) return { package: slash[1], message: slash[2] };

  // Fall back: try catalog message name contained in schema/msg_type/name.
  const hay = `${raw} ${channel.name}`.toLowerCase();
  const hit = MSG_CATALOG.find((m) => hay.includes(m.message.toLowerCase()));
  if (hit) return { package: hit.package, message: hit.message };

  return { package: 'std_msgs', message: 'String' };
}

export function defaultPropsFor(def: DisplayTypeDef, nameHint?: string): Record<string, PropValue> {
  const out: Record<string, PropValue> = {};
  for (const p of def.props) {
    out[p.key] = p.key === 'name' ? nameHint ?? def.label : p.defaultValue;
  }
  return out;
}

/** Messages under a package that are good Add targets (prefer curated, else all). */
export function messagesForPackage(packageName: string): MsgCatalogEntry[] {
  const curated = DISPLAY_TYPE_DEFS.filter((d) => d.package === packageName).map((d) => ({
    package: d.package,
    message: d.message,
    file: `${d.package}/${d.message.toLowerCase()}.proto`,
  }));
  if (curated.length) {
    const curatedMsgs = new Set(curated.map((c) => c.message));
    const rest = MSG_CATALOG.filter(
      (m) => m.package === packageName && !curatedMsgs.has(m.message),
    );
    return [...curated, ...rest];
  }
  return MSG_CATALOG.filter((m) => m.package === packageName);
}
