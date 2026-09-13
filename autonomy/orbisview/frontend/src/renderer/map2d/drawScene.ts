import { makeWorldToScreen } from './coords';
import { drawOccupancyGrid } from './drawOccupancy';
import { drawFootprint, resolveFootprintPoints } from './drawFootprint';
import { drawMapHud } from './drawHud';
import type {
  ChassisJson,
  DefaultFootprint,
  LayerFlags,
  OccupancyGridJson,
  Pose2D,
  RobotFootprintJson,
} from './types';

export interface Map2DSceneInput {
  width: number;
  height: number;
  scale: number;
  viewOffset: { x: number; y: number };
  layers: LayerFlags;
  pose: Pose2D | null;
  path: { poses: Pose2D[] } | null;
  map: OccupancyGridJson | null;
  costmap: OccupancyGridJson | null;
  laser: { angle_min: number; angle_increment: number; ranges: number[] } | null;
  tf: { transforms: { parent: string; child: string; x: number; y: number; yaw?: number }[] } | null;
  obstacles: {
    obstacles?: {
      id: number;
      x: number;
      y: number;
      yaw?: number;
      length?: number;
      width?: number;
      type?: string;
    }[];
  } | null;
  vectorMap: {
    lanes?: { id: string; points: number[][] }[];
    keepouts?: { id: string; polygon: number[][] }[];
  } | null;
  prediction: {
    obstacles?: { id: number; trajectory?: { x: number; y: number }[] }[];
  } | null;
  goal: Pose2D | null;
  twist: { vx: number; wz: number } | null;
  chassis: ChassisJson | null;
  footprint: RobotFootprintJson | null;
  defaultFootprint: DefaultFootprint;
  measurePts: { x: number; y: number }[];
}

export function paintMap2DScene(
  ctx: CanvasRenderingContext2D,
  input: Map2DSceneInput,
): void {
  const {
    width: w,
    height: h,
    scale,
    viewOffset,
    layers,
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
    defaultFootprint,
    measurePts,
  } = input;

  const ox = viewOffset.x;
  const oy = viewOffset.y;
  const toScreen = makeWorldToScreen(w, h, ox, oy, scale);

  ctx.fillStyle = '#0f1419';
  ctx.fillRect(0, 0, w, h);

  if (layers.grid) {
    ctx.strokeStyle = '#1f2a33';
    ctx.lineWidth = 1;
    for (let i = -10; i <= 10; i++) {
      const [x0, y0] = toScreen(ox + i, oy - 10);
      const [x1, y1] = toScreen(ox + i, oy + 10);
      ctx.beginPath();
      ctx.moveTo(x0, y0);
      ctx.lineTo(x1, y1);
      ctx.stroke();
      const [a0, b0] = toScreen(ox - 10, oy + i);
      const [a1, b1] = toScreen(ox + 10, oy + i);
      ctx.beginPath();
      ctx.moveTo(a0, b0);
      ctx.lineTo(a1, b1);
      ctx.stroke();
    }
  }

  if (layers.map && map) {
    drawOccupancyGrid(ctx, map, toScreen, scale, 'map');
  }
  if (layers.costmap && costmap) {
    drawOccupancyGrid(ctx, costmap, toScreen, scale, 'costmap');
  }

  if (layers.vectormap && vectorMap) {
    ctx.strokeStyle = '#546e7a';
    ctx.lineWidth = 1.5;
    for (const lane of vectorMap.lanes ?? []) {
      ctx.beginPath();
      lane.points.forEach((p, i) => {
        const [sx, sy] = toScreen(p[0], p[1]);
        if (i === 0) ctx.moveTo(sx, sy);
        else ctx.lineTo(sx, sy);
      });
      ctx.stroke();
    }
    ctx.strokeStyle = '#ef5350';
    for (const k of vectorMap.keepouts ?? []) {
      ctx.beginPath();
      k.polygon.forEach((p, i) => {
        const [sx, sy] = toScreen(p[0], p[1]);
        if (i === 0) ctx.moveTo(sx, sy);
        else ctx.lineTo(sx, sy);
      });
      ctx.closePath();
      ctx.stroke();
    }
  }

  if (layers.obstacles && obstacles?.obstacles) {
    for (const o of obstacles.obstacles) {
      const yaw = o.yaw ?? 0;
      const L = (o.length ?? 0.6) / 2;
      const W = (o.width ?? 0.4) / 2;
      const c = Math.cos(yaw);
      const s = Math.sin(yaw);
      const corners: [number, number][] = [
        [L, W],
        [L, -W],
        [-L, -W],
        [-L, W],
      ];
      ctx.strokeStyle = o.type === 'PEDESTRIAN' ? '#ffeb3b' : '#ff8a65';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      corners.forEach(([lx, ly], i) => {
        const wx = o.x + lx * c - ly * s;
        const wy = o.y + lx * s + ly * c;
        const [sx, sy] = toScreen(wx, wy);
        if (i === 0) ctx.moveTo(sx, sy);
        else ctx.lineTo(sx, sy);
      });
      ctx.closePath();
      ctx.stroke();
    }
  }

  if (layers.prediction && prediction?.obstacles) {
    ctx.strokeStyle = '#81d4fa';
    ctx.setLineDash([3, 3]);
    ctx.lineWidth = 1;
    for (const o of prediction.obstacles) {
      const traj = o.trajectory ?? [];
      if (traj.length < 2) continue;
      ctx.beginPath();
      traj.forEach((p, i) => {
        const [sx, sy] = toScreen(p.x, p.y);
        if (i === 0) ctx.moveTo(sx, sy);
        else ctx.lineTo(sx, sy);
      });
      ctx.stroke();
    }
    ctx.setLineDash([]);
  }

  if (layers.path && path?.poses?.length) {
    ctx.strokeStyle = '#4fc3f7';
    ctx.lineWidth = 2;
    ctx.beginPath();
    path.poses.forEach((p, i) => {
      const [sx, sy] = toScreen(p.x, p.y);
      if (i === 0) ctx.moveTo(sx, sy);
      else ctx.lineTo(sx, sy);
    });
    ctx.stroke();
  }

  if (layers.laser && laser && pose) {
    ctx.fillStyle = '#ffeb3b';
    laser.ranges.forEach((r, i) => {
      const a =
        (laser.angle_min ?? 0) +
        i * (laser.angle_increment ?? 0) +
        (pose.yaw ?? 0);
      const [sx, sy] = toScreen(
        pose.x + r * Math.cos(a),
        pose.y + r * Math.sin(a),
      );
      ctx.fillRect(sx - 1, sy - 1, 2, 2);
    });
  }

  if (layers.tf && tf?.transforms?.length) {
    ctx.strokeStyle = '#ce93d8';
    ctx.fillStyle = '#ce93d8';
    ctx.lineWidth = 1.5;
    for (const t of tf.transforms) {
      const [sx, sy] = toScreen(t.x, t.y);
      const yaw = t.yaw ?? 0;
      const len = 14;
      const ex = sx + Math.cos(yaw) * len;
      const ey = sy - Math.sin(yaw) * len;
      ctx.beginPath();
      ctx.moveTo(sx, sy);
      ctx.lineTo(ex, ey);
      ctx.stroke();
      ctx.beginPath();
      ctx.arc(sx, sy, 2.5, 0, Math.PI * 2);
      ctx.fill();
      ctx.font = '10px sans-serif';
      ctx.fillText(t.child, sx + 4, sy - 4);
    }
  }

  if (goal) {
    const [sx, sy] = toScreen(goal.x, goal.y);
    ctx.strokeStyle = '#ff7043';
    ctx.fillStyle = '#ff7043';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(sx, sy - 10);
    ctx.lineTo(sx + 8, sy + 6);
    ctx.lineTo(sx - 8, sy + 6);
    ctx.closePath();
    ctx.fill();
    if (pose) {
      ctx.setLineDash([4, 4]);
      ctx.beginPath();
      const [rx, ry] = toScreen(pose.x, pose.y);
      ctx.moveTo(rx, ry);
      ctx.lineTo(sx, sy);
      ctx.stroke();
      ctx.setLineDash([]);
    }
  }

  const footprintSource: 'stream' | 'default' = footprint ? 'stream' : 'default';
  if (layers.footprint && pose) {
    const pts = resolveFootprintPoints(footprint, defaultFootprint);
    drawFootprint(ctx, pose, pts, toScreen);
  }

  if (layers.robot && pose) {
    const [sx, sy] = toScreen(pose.x, pose.y);
    ctx.fillStyle = '#69f0ae';
    ctx.beginPath();
    ctx.arc(sx, sy, 6, 0, Math.PI * 2);
    ctx.fill();
    const yaw = pose.yaw ?? 0;
    ctx.strokeStyle = '#69f0ae';
    ctx.beginPath();
    ctx.moveTo(sx, sy);
    ctx.lineTo(sx + Math.cos(yaw) * 18, sy - Math.sin(yaw) * 18);
    ctx.stroke();
  }

  drawMapHud(ctx, {
    pose,
    twist,
    chassis,
    goal,
    footprintSource,
  });

  if (measurePts.length) {
    ctx.strokeStyle = '#ffee58';
    ctx.fillStyle = '#ffee58';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    measurePts.forEach((p, i) => {
      const [sx, sy] = toScreen(p.x, p.y);
      if (i === 0) ctx.moveTo(sx, sy);
      else ctx.lineTo(sx, sy);
      ctx.fillRect(sx - 2, sy - 2, 4, 4);
    });
    ctx.stroke();
  }
}
