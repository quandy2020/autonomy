import type { OccupancyGridJson, Pose2D, RobotFootprintJson } from '../map2d/types';
import type { StaticSlamCanvasHandle } from '../map2d/staticSlam';
import type { SemanticZoneNorm } from '../map2d/semanticZones';

export type CloudColorMode = 'height' | 'intensity';

export interface View3DOpts {
  cloudColor: CloudColorMode;
  cloudSize?: number;
  laserHeight: number;
  laserColor?: number;
  laserSize?: number;
  mapOpacity: number;
  pathColor?: number;
  pathOpacity?: number;
  pathLineWidth?: number;
}

export interface View3DLayerFlags {
  grid: boolean;
  basemap?: boolean;
  semantic?: boolean;
  robot: boolean;
  path: boolean;
  pointcloud: boolean;
  footprint: boolean;
  map: boolean;
  costmap: boolean;
  laser: boolean;
}

export interface View3DCloudPoint {
  x: number;
  y: number;
  z: number;
  i?: number;
}

export interface View3DLaserScan {
  angle_min: number;
  angle_increment: number;
  ranges: number[];
}

export interface View3DNavGoal {
  x: number;
  y: number;
}

export interface View3DWaypoint {
  id: string;
  x: number;
  y: number;
  yaw?: number;
  label?: string;
  color: string;
  selected: boolean;
}

export interface View3DSceneInput {
  pose: Pose2D | null;
  path: Pose2D[] | null;
  goal: View3DNavGoal | null;
  waypoints: View3DWaypoint[];
  cloud: View3DCloudPoint[] | null;
  footprint: RobotFootprintJson | null;
  map: OccupancyGridJson | null;
  costmap: OccupancyGridJson | null;
  basemap?: StaticSlamCanvasHandle | null;
  semanticZones?: SemanticZoneNorm[] | null;
  laser: View3DLaserScan | null;
  layers: View3DLayerFlags;
  opts: View3DOpts;
  followRobot: boolean;
}
