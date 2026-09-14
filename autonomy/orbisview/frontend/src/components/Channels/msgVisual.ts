import type { IconName } from '@/components/icons';

/** Packages / names that are metadata or scalars — show payload as JSON, not geometry. */
const META_PACKAGES = new Set([
  'std_msgs',
  'builtin_interfaces',
  'unique_identifier_msgs',
]);

const META_NAME_RE =
  /(MetaData|Metadata|Header$|^Time$|^Duration$|MultiArrayDimension|MultiArrayLayout|PointField|RegionOfInterest|^UUID$|ColorRGBA|^Empty$)/i;

export function isMetaMessage(packageName: string, message: string): boolean {
  if (META_PACKAGES.has(packageName)) return true;
  if (META_NAME_RE.test(message)) return true;
  return false;
}

export function isMetaTypeId(typeId: string): boolean {
  const [pkg, msg] = typeId.split('/');
  return isMetaMessage(pkg ?? '', msg ?? typeId);
}

const EXACT: Record<string, IconName> = {
  'sensor_msgs/LaserScan': 'laser',
  'sensor_msgs/PointCloud2': 'pointcloud',
  'sensor_msgs/PointCloud': 'pointcloud',
  'sensor_msgs/Image': 'image',
  'sensor_msgs/CompressedImage': 'image',
  'sensor_msgs/CameraInfo': 'image',
  'sensor_msgs/Depth': 'depth',
  'stereo_msgs/DisparityImage': 'depth',
  'nav_msgs/Path': 'path',
  'nav_msgs/Odometry': 'pose',
  'nav_msgs/OccupancyGrid': 'costmap',
  'nav_msgs/Route': 'route',
  'nav_msgs/Goals': 'goal',
  'map_msgs/OccupancyGrid': 'costmap',
  'map_msgs/GridMap': 'grid',
  'map_msgs/Octomap': 'layers',
  'tf2_msgs/TFMessage': 'tf',
  'geometry_msgs/Pose': 'pose',
  'geometry_msgs/Pose2D': 'pose',
  'geometry_msgs/PoseStamped': 'pose',
  'geometry_msgs/PoseArray': 'pose',
  'geometry_msgs/Twist': 'velocity',
  'geometry_msgs/TwistStamped': 'velocity',
  'geometry_msgs/Polygon': 'footprint',
  'geometry_msgs/PolygonStamped': 'footprint',
  'geometry_msgs/Transform': 'tf',
  'geometry_msgs/TransformStamped': 'tf',
  'geometry_msgs/Point': 'pick',
  'geometry_msgs/PointStamped': 'pick',
  'visualization_msgs/Marker': 'obstacle',
  'visualization_msgs/MarkerArray': 'obstacle',
  'vehicle_msgs/RobotState': 'robot',
  'vehicle_msgs/RobotEvent': 'chassis',
  'status_msgs/Status': 'diagnostics',
  'diagnostic_msgs/DiagnosticArray': 'diagnostics',
  'diagnostic_msgs/DiagnosticStatus': 'diagnostics',
  'audio_msgs/AudioData': 'audio',
  'audio_msgs/AudioDataStamped': 'audio',
  'audio_msgs/AudioInfo': 'audio',
};

const PACKAGE_ICON: Record<string, IconName> = {
  sensor_msgs: 'laser',
  nav_msgs: 'nav',
  map_msgs: 'map2d',
  geometry_msgs: 'pose',
  visualization_msgs: 'obstacle',
  vision_msgs: 'inspector',
  tf2_msgs: 'tf',
  trajectory_msgs: 'path',
  vehicle_msgs: 'chassis',
  status_msgs: 'diagnostics',
  diagnostic_msgs: 'diagnostics',
  audio_msgs: 'audio',
  std_msgs: 'json',
  builtin_interfaces: 'json',
  unique_identifier_msgs: 'json',
  action_msgs: 'goal',
  shape_msgs: 'mesh',
  pcl_msgs: 'pointcloud',
  stereo_msgs: 'depth',
  strata_msgs: 'layers',
};

function heuristicIcon(message: string): IconName | null {
  const m = message.toLowerCase();
  if (/(laserscan|range|radar)/.test(m)) return 'laser';
  if (/pointcloud|point_indices|vertices/.test(m)) return 'pointcloud';
  if (/image|camera|compressed/.test(m)) return 'image';
  if (/depth|disparity/.test(m)) return 'depth';
  if (/occupancy|costmap|grid_cells|voxel|octomap|grid_map|mapmeta|projected_map/.test(m))
    return 'costmap';
  if (/path|trajectory|route/.test(m)) return 'path';
  if (/odom|pose|transform|tf/.test(m)) return m.includes('tf') || m.includes('transform') ? 'tf' : 'pose';
  if (/twist|velocity|speed/.test(m)) return 'velocity';
  if (/polygon|footprint|mesh|solid_primitive|plane/.test(m)) return 'footprint';
  if (/marker|obstacle|detection|bounding|hypothesis|classification/.test(m)) return 'obstacle';
  if (/goal|waypoint|nav/.test(m)) return 'goal';
  if (/audio/.test(m)) return 'audio';
  if (/diagnostic|status|event/.test(m)) return 'diagnostics';
  if (/robot|chassis|vehicle|task/.test(m)) return 'robot';
  if (/point|vector|quaternion/.test(m)) return 'pick';
  if (/time|duration|header|uuid|string|bool|byte|char|float|int|uint|empty|color|array|meta/.test(m))
    return 'json';
  return null;
}

export function msgIcon(packageName: string, message: string): IconName {
  const typeId = `${packageName}/${message}`;
  if (isMetaMessage(packageName, message)) return 'json';
  if (EXACT[typeId]) return EXACT[typeId];
  return heuristicIcon(message) ?? PACKAGE_ICON[packageName] ?? 'msg';
}

export function packageIcon(packageName: string): IconName {
  return PACKAGE_ICON[packageName] ?? 'msg';
}

/** Pretty JSON for meta displays; truncates huge payloads. */
export function formatPayloadJson(payload: unknown, maxChars = 8000): string {
  try {
    const text = JSON.stringify(payload ?? null, null, 2);
    if (text.length <= maxChars) return text;
    return `${text.slice(0, maxChars)}\n… truncated (${text.length} chars)`;
  } catch {
    return String(payload);
  }
}
