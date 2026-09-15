export type WsOp =
  | 'list_channels'
  | 'subscribe'
  | 'unsubscribe'
  | 'status'
  | 'channels'
  | 'subscribed'
  | 'unsubscribed'
  | 'envelope'
  | 'error';

export interface ChannelInfo {
  name: string;
  schema: string;
  msg_type: string;
  has_writer: boolean;
  mock: boolean;
  /** Numeric protobuf field paths for Charts (from descriptor). */
  fields?: string[];
}

/** Autolink service transport + Action /feedback|/status (not user topics). */
export function isBrowsableChannel(name: string, msgType?: string): boolean {
  if (name.includes('_SRV_')) return false;
  if (name.endsWith('/feedback') || name.endsWith('/status')) return false;
  if (msgType && /FeedbackMessage|StatusMessage/i.test(msgType)) return false;
  return true;
}

export interface StreamEnvelope {
  op: 'envelope';
  channel: string;
  schema: string;
  timestamp: number;
  frame_id: string;
  sequence: number;
  encoding: string;
  unsupported?: boolean;
  stale?: boolean;
  payload?: unknown;
  payload_b64?: string;
}

export type ServerMessage =
  | { op: 'channels'; channels: ChannelInfo[] }
  | { op: 'subscribed'; channel: string; max_hz: number }
  | { op: 'unsubscribed'; channel: string }
  | { op: 'status'; clients: number; dropped_frames: number; mock: boolean; autolink: boolean; transport?: string }
  | { op: 'error'; message: string }
  | { op: 'channel_stats'; channels: { name: string; count: number; hz: number; latency_ms: number }[] }
  | {
      op: 'plugins';
      plugins: {
        id: string;
        kind: string;
        title: string;
        version: string;
        enabled: boolean;
        source?: string;
        path?: string;
      }[];
      failures?: string[];
    }
  | {
      op: 'plugin_host';
      loaded: { id: string; path: string; kind: string; version: string }[];
      failures?: string[];
    }
  | {
      op: 'recorder_status';
      recording: boolean;
      playing: boolean;
      paused?: boolean;
      recorded: number;
      play_index?: number;
      play_total?: number;
      speed?: number;
      loop?: boolean;
      record_path: string;
      play_path: string;
    }
  | { op: 'bag_index'; path: string; count: number; entries: unknown[] }
  | { op: 'goal_set'; x: number; y: number; mock?: boolean }
  | { op: 'goal_cleared' }
  | { op: 'cmd_vel_ack'; vx: number; wz: number }
  | { op: 'route_set'; count: number }
  | { op: 'route_cleared' }
  | { op: 'hmi_mode'; mode: string }
  | { op: 'hmi_module_ack' }
  | { op: 'hmi_status'; status: unknown; components: unknown }
  | { op: 'dump_ok'; path: string }
  | { op: 'sim_cleared' }
  | { op: 'local_bags'; path: string; bags: { name: string; path: string }[] }
  | StreamEnvelope;

export const SCHEMAS = {
  Pose2D: 'orbisview.render.Pose2D',
  Path2D: 'orbisview.render.Path2D',
  OccupancyGrid: 'orbisview.render.OccupancyGrid',
  RobotFootprint: 'orbisview.render.RobotFootprint',
  TfTree: 'orbisview.render.TfTree',
  LaserScan: 'orbisview.render.LaserScan',
  Image: 'orbisview.render.Image',
  PointCloud2: 'orbisview.render.PointCloud2',
  DepthImage: 'orbisview.render.DepthImage',
  Exploration: 'orbisview.render.Exploration',
  Navigation: 'orbisview.render.Navigation',
  Mapping: 'orbisview.render.Mapping',
  SemanticZoneArray: 'orbisview.render.SemanticZoneArray',
  FloorInfoArray: 'orbisview.render.FloorInfoArray',
  Twist2D: 'orbisview.render.Twist2D',
  ChassisState: 'orbisview.render.ChassisState',
  ObstacleArray: 'orbisview.render.ObstacleArray',
  WorldState: 'orbisview.render.WorldState',
  RoutePath: 'orbisview.render.RoutePath',
  VectorMap: 'orbisview.render.VectorMap',
  PredictionObstacles: 'orbisview.render.PredictionObstacles',
  PlanningDebug: 'orbisview.render.PlanningDebug',
  HmiStatus: 'orbisview.render.HmiStatus',
  ComponentsStatus: 'orbisview.render.ComponentsStatus',
} as const;

/** Mark envelope stale if older than this (wall clock vs payload timestamp ns). */
export const STALE_THRESHOLD_MS = 2000;
