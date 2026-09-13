import type { OccupancyGridJson, Pose2D, RobotFootprintJson } from '../map2d/types';

export type CloudColorMode = 'height' | 'intensity';

export interface View3DOpts {
  cloudColor: CloudColorMode;
  laserHeight: number;
  mapOpacity: number;
}

export interface View3DLayerFlags {
  grid: boolean;
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

export interface View3DSceneInput {
  pose: Pose2D | null;
  path: Pose2D[] | null;
  goal: View3DNavGoal | null;
  cloud: View3DCloudPoint[] | null;
  footprint: RobotFootprintJson | null;
  map: OccupancyGridJson | null;
  costmap: OccupancyGridJson | null;
  laser: View3DLaserScan | null;
  layers: View3DLayerFlags;
  opts: View3DOpts;
  followRobot: boolean;
}
