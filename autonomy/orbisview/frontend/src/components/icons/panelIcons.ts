import type { IconName } from './Icon';
import { panelBaseId } from '@/components/panelId';

/** Panel id → icon for catalog and mosaic titles. */
export const PANEL_ICONS: Record<string, IconName> = {
  map2d: 'map2d',
  map: 'map2d',
  dashboard: 'dashboard',
  mode_settings: 'mode',
  module_delay: 'delay',
  resources: 'resources',
  robot_status: 'robot',
  tf_tree: 'tf',
  waypoints: 'waypoint',
  routing: 'route',
  teleop: 'teleop',
  ops: 'ops',
  view3d: 'view3d',
  image: 'image',
  log: 'channels',
  inspector: 'inspector',
  diagnostics: 'diagnostics',
  stats: 'stats',
  recorder: 'recorder',
  plugins: 'plugins',
  pnc: 'pnc',
  charts: 'charts',
  components: 'components',
  hmi: 'hmi',
  exploration: 'explore',
  navigation: 'nav',
  mapping: 'mapping',
};

export const LAYER_ICONS: Record<string, IconName> = {
  grid: 'grid',
  map: 'map2d',
  costmap: 'costmap',
  vectormap: 'route',
  path: 'path',
  robot: 'robot',
  footprint: 'footprint',
  obstacles: 'obstacle',
  prediction: 'prediction',
  laser: 'laser',
  tf: 'tf',
  pointcloud: 'pointcloud',
  image: 'image',
  depth: 'depth',
};

export function panelIcon(id: string): IconName {
  return PANEL_ICONS[panelBaseId(id)] ?? 'panels';
}

export function layerIcon(key: string): IconName {
  return LAYER_ICONS[key] ?? 'layers';
}
