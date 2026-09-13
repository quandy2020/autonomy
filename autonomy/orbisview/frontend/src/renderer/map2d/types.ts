export interface Pose2D {
  x: number;
  y: number;
  yaw?: number;
}

export interface OccupancyGridJson {
  resolution: number;
  width: number;
  height: number;
  origin: Pose2D;
  data: number[];
}

export interface RobotFootprintJson {
  shape?: string | number;
  points?: Pose2D[];
  radius?: number;
  padding?: number;
  length?: number;
  width?: number;
}

export interface ChassisJson {
  vx?: number;
  wz?: number;
  steering?: number;
  motion_model?: string | number;
  driving_mode?: string;
}

export interface LayerFlags {
  grid: boolean;
  map: boolean;
  costmap: boolean;
  vectormap: boolean;
  path: boolean;
  robot: boolean;
  footprint: boolean;
  obstacles: boolean;
  prediction: boolean;
  laser: boolean;
  tf: boolean;
}

export interface DefaultFootprint {
  shape: string;
  length: number;
  width: number;
}
