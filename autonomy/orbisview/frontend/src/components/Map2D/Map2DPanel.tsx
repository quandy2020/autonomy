import { useEffect, useMemo, useRef, useState } from 'react';
import { DEFAULT_FOOTPRINT } from '../../../config/parameters.js';
import { paintMap2DScene } from '@/renderer';
import type {
  ChassisJson,
  OccupancyGridJson,
  Pose2D,
  RobotFootprintJson,
} from '@/renderer/map2d/types';
import { useDataStore } from '@/store/dataStore';
import { useLayerStore } from '@/store/layoutStore';
import { useWaypointStore } from '@/store/waypointStore';
import { SCHEMAS, type StreamEnvelope } from '@/store/websocket/types';
import { wsClient } from '@/store/websocket/client';

interface Path2D {
  poses: Pose2D[];
}

interface LaserScan {
  angle_min: number;
  angle_increment: number;
  ranges: number[];
}

interface NavPayload {
  state?: string;
  goal?: Pose2D;
  has_goal?: boolean;
  distance_remaining?: number;
}

interface Twist2D {
  vx: number;
  wz: number;
}

interface TfTree {
  transforms: {
    parent: string;
    child: string;
    x: number;
    y: number;
    yaw?: number;
  }[];
}

function asPayload<T>(env: { payload?: unknown } | undefined): T | null {
  if (!env?.payload || typeof env.payload !== 'object') return null;
  return env.payload as T;
}

function pickMapAndCostmap(envelopes: Record<string, StreamEnvelope>) {
  const grids = Object.values(envelopes).filter(
    (e) => e.schema === SCHEMAS.OccupancyGrid,
  );
  const costmapEnv = grids.find((e) => /costmap/i.test(e.channel));
  const mapEnv =
    grids.find((e) => e !== costmapEnv && !/costmap/i.test(e.channel)) ??
    grids.find((e) => e !== costmapEnv);
  return {
    map: asPayload<OccupancyGridJson>(mapEnv),
    costmap: asPayload<OccupancyGridJson>(costmapEnv),
  };
}

const SCALE = 40;

export function Map2DPanel() {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const envelopes = useDataStore((s) => s.envelopes);
  const connected = useDataStore((s) => s.connected);
  const layers = useLayerStore();
  const followRobot = useLayerStore((s) => s.followRobot);
  const setFollowRobot = useLayerStore((s) => s.setFollowRobot);
  const addWaypoint = useWaypointStore((s) => s.add);
  const [localGoal, setLocalGoal] = useState<Pose2D | null>(null);
  const [viewOffset, setViewOffset] = useState({ x: 0, y: 0 });

  const [mapTool, setMapTool] = useState<'goal' | 'measure' | 'copy'>('goal');
  const [measurePts, setMeasurePts] = useState<{ x: number; y: number }[]>([]);
  const [measureMsg, setMeasureMsg] = useState('');

  const pose = useMemo(() => {
    const e = Object.values(envelopes).find((x) => x.schema === SCHEMAS.Pose2D);
    return asPayload<Pose2D>(e);
  }, [envelopes]);

  const path = useMemo(() => {
    const e = Object.values(envelopes).find((x) => x.schema === SCHEMAS.Path2D);
    return asPayload<Path2D>(e);
  }, [envelopes]);

  const { map, costmap } = useMemo(
    () => pickMapAndCostmap(envelopes),
    [envelopes],
  );

  const laser = useMemo(() => {
    const e = Object.values(envelopes).find((x) => x.schema === SCHEMAS.LaserScan);
    return asPayload<LaserScan>(e);
  }, [envelopes]);

  const nav = useMemo(() => {
    const e = Object.values(envelopes).find((x) => x.schema === SCHEMAS.Navigation);
    return asPayload<NavPayload>(e);
  }, [envelopes]);

  const twist = useMemo(() => {
    const e = Object.values(envelopes).find((x) => x.schema === SCHEMAS.Twist2D);
    return asPayload<Twist2D>(e);
  }, [envelopes]);

  const chassis = useMemo(() => {
    const e = Object.values(envelopes).find(
      (x) => x.schema === SCHEMAS.ChassisState,
    );
    return asPayload<ChassisJson>(e);
  }, [envelopes]);

  const footprint = useMemo(() => {
    const e = Object.values(envelopes).find(
      (x) => x.schema === SCHEMAS.RobotFootprint,
    );
    return asPayload<RobotFootprintJson>(e);
  }, [envelopes]);

  const tf = useMemo(() => {
    const e = Object.values(envelopes).find((x) => x.schema === SCHEMAS.TfTree);
    return asPayload<TfTree>(e);
  }, [envelopes]);

  const obstacles = useMemo(() => {
    const e = Object.values(envelopes).find((x) => x.schema === SCHEMAS.ObstacleArray);
    return asPayload<{
      obstacles?: {
        id: number;
        x: number;
        y: number;
        yaw?: number;
        length?: number;
        width?: number;
        type?: string;
      }[];
    }>(e);
  }, [envelopes]);

  const vectorMap = useMemo(() => {
    const e = Object.values(envelopes).find((x) => x.schema === SCHEMAS.VectorMap);
    return asPayload<{
      lanes?: { id: string; points: number[][] }[];
      keepouts?: { id: string; polygon: number[][] }[];
    }>(e);
  }, [envelopes]);

  const prediction = useMemo(() => {
    const e = Object.values(envelopes).find(
      (x) => x.schema === SCHEMAS.PredictionObstacles,
    );
    return asPayload<{
      obstacles?: { id: number; trajectory?: { x: number; y: number }[] }[];
    }>(e);
  }, [envelopes]);

  const goal =
    localGoal ??
    (nav?.has_goal !== false && nav?.goal ? nav.goal : null);

  useEffect(() => {
    if (followRobot && pose) {
      setViewOffset({ x: pose.x, y: pose.y });
    }
  }, [followRobot, pose]);

  const anyStale = Object.values(envelopes).some(
    (e) =>
      e.stale &&
      (e.schema === SCHEMAS.Pose2D ||
        e.schema === SCHEMAS.Path2D ||
        e.schema === SCHEMAS.OccupancyGrid ||
        e.schema === SCHEMAS.LaserScan ||
        e.schema === SCHEMAS.RobotFootprint),
  );

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    paintMap2DScene(ctx, {
      width: canvas.width,
      height: canvas.height,
      scale: SCALE,
      viewOffset,
      layers: {
        grid: layers.grid,
        map: layers.map,
        costmap: layers.costmap,
        vectormap: layers.vectormap,
        path: layers.path,
        robot: layers.robot,
        footprint: layers.footprint,
        obstacles: layers.obstacles,
        prediction: layers.prediction,
        laser: layers.laser,
        tf: layers.tf,
      },
      pose,
      path,
      map,
      costmap,
      laser,
      tf,
      obstacles,
      vectorMap,
      prediction,
      goal,
      twist,
      chassis,
      footprint,
      defaultFootprint: DEFAULT_FOOTPRINT,
      measurePts,
    });
  }, [
    pose,
    path,
    map,
    costmap,
    laser,
    layers,
    goal,
    twist,
    chassis,
    footprint,
    tf,
    viewOffset,
    obstacles,
    vectorMap,
    prediction,
    measurePts,
  ]);

  const worldFromEvent = (e: React.MouseEvent<HTMLCanvasElement>) => {
    const canvas = canvasRef.current;
    if (!canvas) return null;
    const rect = canvas.getBoundingClientRect();
    const sx = ((e.clientX - rect.left) / rect.width) * canvas.width;
    const sy = ((e.clientY - rect.top) / rect.height) * canvas.height;
    const x = (sx - canvas.width / 2) / SCALE + viewOffset.x;
    const y = (canvas.height / 2 - sy) / SCALE + viewOffset.y;
    return { x, y };
  };

  const onClick = (e: React.MouseEvent<HTMLCanvasElement>) => {
    if (!connected && mapTool === 'goal') return;
    const w = worldFromEvent(e);
    if (!w) return;
    if (mapTool === 'goal') {
      setLocalGoal(w);
      wsClient.send({ op: 'set_goal', x: w.x, y: w.y });
      return;
    }
    if (mapTool === 'copy') {
      const text = `${w.x.toFixed(4)},${w.y.toFixed(4)}`;
      void navigator.clipboard?.writeText(text);
      setMeasureMsg(`copied ${text}`);
      return;
    }
    const next = [...measurePts, w];
    setMeasurePts(next);
    if (next.length >= 2) {
      let dist = 0;
      for (let i = 1; i < next.length; i++) {
        const dx = next[i].x - next[i - 1].x;
        const dy = next[i].y - next[i - 1].y;
        dist += Math.hypot(dx, dy);
      }
      setMeasureMsg(`path ${dist.toFixed(2)} m · ${next.length} pts (RMB finish)`);
    } else {
      setMeasureMsg('measure: click more points, RMB to finish');
    }
  };

  const onContextMenu = (e: React.MouseEvent<HTMLCanvasElement>) => {
    e.preventDefault();
    const w = worldFromEvent(e);
    if (!w) return;
    if (mapTool === 'measure') {
      setMeasureMsg((m) => `${m} · done`);
      return;
    }
    addWaypoint(w.x, w.y);
  };

  const clearGoal = () => {
    setLocalGoal(null);
    wsClient.send({ op: 'clear_goal' });
  };

  return (
    <div className="panel map-primary">
      {anyStale ? <div className="stale-badge">map data stale</div> : null}
      <div className="row map-toolbar" style={{ flexWrap: 'wrap', gap: 8 }}>
        <label className="row">
          <input
            type="checkbox"
            checked={followRobot}
            onChange={(e) => setFollowRobot(e.target.checked)}
          />
          Follow robot
        </label>
        <button
          type="button"
          className={mapTool === 'goal' ? 'tab active' : 'tab'}
          onClick={() => setMapTool('goal')}
        >
          Goal
        </button>
        <button
          type="button"
          className={mapTool === 'measure' ? 'tab active' : 'tab'}
          onClick={() => {
            setMapTool('measure');
            setMeasurePts([]);
            setMeasureMsg('measure mode');
          }}
        >
          Measure
        </button>
        <button
          type="button"
          className={mapTool === 'copy' ? 'tab active' : 'tab'}
          onClick={() => setMapTool('copy')}
        >
          Copy XY
        </button>
        <button
          type="button"
          onClick={() => {
            setMeasurePts([]);
            setMeasureMsg('');
          }}
        >
          Clear measure
        </button>
        <span className="hint">{measureMsg || 'LMB tool · RMB waypoint'}</span>
      </div>
      <canvas
        ref={canvasRef}
        width={960}
        height={640}
        className="map-canvas map-canvas-fill"
        onClick={onClick}
        onContextMenu={onContextMenu}
        style={{ cursor: connected || mapTool !== 'goal' ? 'crosshair' : 'default' }}
      />
      <div className="row" style={{ gap: 8, marginTop: 8 }}>
        <button type="button" onClick={clearGoal} disabled={!connected}>
          Clear goal
        </button>
        {goal ? (
          <span className="muted">
            goal ({goal.x.toFixed(2)}, {goal.y.toFixed(2)})
            {nav?.distance_remaining != null
              ? ` · rem=${nav.distance_remaining.toFixed(2)}`
              : ''}
          </span>
        ) : null}
      </div>
    </div>
  );
}
