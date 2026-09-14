import { makeWorldToScreen } from './coords';
import { drawOccupancyGrid } from './drawOccupancy';
import { drawFootprint, resolveFootprintPoints } from './drawFootprint';
import { drawSemanticZones } from './drawSemantic';
import type { SemanticZoneNorm } from './semanticZones';
import type {
  DefaultFootprint,
  LayerFlags,
  OccupancyGridJson,
  Pose2D,
  RobotFootprintJson,
} from './types';

function strokeFromHex(hex: string, alpha: number): string {
  const h = hex.replace('#', '');
  const full = h.length === 3 ? h.split('').map((c) => c + c).join('') : h;
  const n = Number.parseInt(full, 16);
  if (!Number.isFinite(n)) return `rgba(79,195,247,${alpha})`;
  return `rgba(${(n >> 16) & 255},${(n >> 8) & 255},${n & 255},${Math.max(0, Math.min(1, alpha))})`;
}

export interface Map2DSceneInput {
  width: number;
  height: number;
  scale: number;
  viewOffset: { x: number; y: number };
  layers: LayerFlags;
  pose: Pose2D | null;
  path: { poses: Pose2D[] } | null;
  pathStyle?: { color: string; lineWidth: number; alpha: number };
  map: OccupancyGridJson | null;
  costmap: OccupancyGridJson | null;
  /** @deprecated prefer lasers[] */
  laser: { angle_min: number; angle_increment: number; ranges: number[] } | null;
  lasers?: {
    scan: { angle_min: number; angle_increment: number; ranges: number[]; range_min?: number; range_max?: number };
    color: string;
    size: number;
    alpha: number;
  }[];
  cloud?: {
    points: { x: number; y: number; z: number; i?: number }[];
    colorMode: 'intensity' | 'height' | 'rgb';
    size: number;
    alpha: number;
  } | null;
  ranges?: {
    range: number;
    field_of_view: number;
    color: string;
    alpha: number;
  }[];
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
  semanticZones?: SemanticZoneNorm[] | null;
  prediction: {
    obstacles?: { id: number; trajectory?: { x: number; y: number }[] }[];
  } | null;
  goal: Pose2D | null;
  footprint: RobotFootprintJson | null;
  defaultFootprint: DefaultFootprint;
  measurePts: { x: number; y: number; yaw?: number; label?: string }[];
  /** Live cursor while measuring (dashed preview to next point). */
  measurePreview?: { x: number; y: number } | null;
  /** Static SLAM / occupancy image basemap (under live map). */
  basemap?: {
    canvas: HTMLCanvasElement;
    originX: number;
    originY: number;
    worldW: number;
    worldH: number;
  } | null;
  /** Nav / multi pose handles: ring + direction arrow. */
  poseHandles?: {
    x: number;
    y: number;
    yaw?: number;
    label?: string;
    /** Ring radius in meters (world). */
    ringM?: number;
    editing?: boolean;
  }[];
  routePts?: {
    x: number;
    y: number;
    yaw?: number;
    label?: string;
    color?: string;
    selected?: boolean;
    editing?: boolean;
  }[];
}

function drawPoseHandle(
  ctx: CanvasRenderingContext2D,
  toScreen: (x: number, y: number) => readonly [number, number],
  scale: number,
  p: {
    x: number;
    y: number;
    yaw?: number;
    label?: string;
    ringM?: number;
    editing?: boolean;
  },
  color: string,
): void {
  const [sx, sy] = toScreen(p.x, p.y);
  const yaw = p.yaw ?? 0;
  const ringM = Math.max(0.28, p.ringM ?? 0.5);
  const ringPx = Math.max(16, ringM * scale);
  const editing = !!p.editing;

  // Outer ring
  ctx.save();
  ctx.strokeStyle = color;
  ctx.fillStyle = color;
  ctx.lineWidth = editing ? 2.4 : 1.6;
  if (editing) ctx.setLineDash([5, 4]);
  ctx.beginPath();
  ctx.arc(sx, sy, ringPx, 0, Math.PI * 2);
  ctx.stroke();
  ctx.setLineDash([]);

  // Soft fill while editing
  if (editing) {
    ctx.fillStyle = color;
    ctx.globalAlpha = 0.12;
    ctx.beginPath();
    ctx.arc(sx, sy, ringPx, 0, Math.PI * 2);
    ctx.fill();
    ctx.globalAlpha = 1;
  }

  // Center
  ctx.beginPath();
  ctx.arc(sx, sy, editing ? 5 : 4, 0, Math.PI * 2);
  ctx.fill();

  // Direction arrow to ring rim
  const ex = sx + Math.cos(yaw) * ringPx;
  const ey = sy - Math.sin(yaw) * ringPx;
  ctx.lineWidth = editing ? 2.4 : 1.8;
  ctx.beginPath();
  ctx.moveTo(sx, sy);
  ctx.lineTo(ex, ey);
  ctx.stroke();

  // Arrow head
  const head = Math.min(14, ringPx * 0.45);
  const ang = Math.atan2(ey - sy, ex - sx);
  ctx.beginPath();
  ctx.moveTo(ex, ey);
  ctx.lineTo(ex - head * Math.cos(ang - 0.5), ey - head * Math.sin(ang - 0.5));
  ctx.lineTo(ex - head * Math.cos(ang + 0.5), ey - head * Math.sin(ang + 0.5));
  ctx.closePath();
  ctx.fill();

  if (p.label) {
    ctx.font = '600 11px ui-sans-serif, system-ui, sans-serif';
    ctx.strokeStyle = 'rgba(11,15,20,0.75)';
    ctx.lineWidth = 3;
    ctx.strokeText(p.label, sx + ringPx + 6, sy - 4);
    ctx.fillStyle = color;
    ctx.fillText(p.label, sx + ringPx + 6, sy - 4);
  }

  if (editing) {
    const deg = (((yaw * 180) / Math.PI) % 360 + 360) % 360;
    const tip = `${deg.toFixed(0)}°`;
    ctx.font = '11px ui-monospace, monospace';
    ctx.strokeStyle = 'rgba(11,15,20,0.8)';
    ctx.lineWidth = 3;
    ctx.strokeText(tip, ex + 8, ey - 4);
    ctx.fillStyle = '#fff59d';
    ctx.fillText(tip, ex + 8, ey - 4);
  }
  ctx.restore();
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
    pathStyle,
    map,
    costmap,
    laser,
    lasers,
    cloud,
    ranges: rangeOverlays,
    tf,
    obstacles,
    vectorMap,
    semanticZones = null,
    prediction,
    goal,
    footprint,
    defaultFootprint,
    measurePts,
    measurePreview = null,
    poseHandles = [],
    routePts = [],
    basemap = null,
  } = input;

  const ox = viewOffset.x;
  const oy = viewOffset.y;
  const toScreen = makeWorldToScreen(w, h, ox, oy, scale);

  ctx.fillStyle = '#141b22';
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

  if ((layers.basemap ?? true) && basemap?.canvas) {
    const x0 = basemap.originX;
    const y0 = basemap.originY;
    const x1 = x0 + basemap.worldW;
    const y1 = y0 + basemap.worldH;
    const [sx0, sy0] = toScreen(x0, y0);
    const [sx1, sy1] = toScreen(x1, y1);
    const left = Math.min(sx0, sx1);
    const top = Math.min(sy0, sy1);
    const bw = Math.abs(sx1 - sx0);
    const bh = Math.abs(sy1 - sy0);
    ctx.imageSmoothingEnabled = false;
    ctx.save();
    ctx.translate(left, top + bh);
    ctx.scale(1, -1);
    ctx.drawImage(basemap.canvas, 0, 0, bw, bh);
    ctx.restore();
  }

  if (layers.map && map) {
    drawOccupancyGrid(ctx, map, toScreen, scale, 'map');
  }
  if (layers.costmap && costmap) {
    drawOccupancyGrid(ctx, costmap, toScreen, scale, 'costmap');
  }

  if ((layers.semantic ?? true) && semanticZones?.length) {
    drawSemanticZones(ctx, semanticZones, toScreen, scale);
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
    const color = pathStyle?.color ?? '#4fc3f7';
    const alpha = pathStyle?.alpha ?? 0.9;
    const lw = pathStyle?.lineWidth ?? 2;
    ctx.strokeStyle = color.includes('rgba') ? color : strokeFromHex(color, alpha);
    ctx.lineWidth = Math.max(1, lw);
    ctx.lineJoin = 'round';
    ctx.lineCap = 'round';
    ctx.beginPath();
    path.poses.forEach((p, i) => {
      const [sx, sy] = toScreen(p.x, p.y);
      if (i === 0) ctx.moveTo(sx, sy);
      else ctx.lineTo(sx, sy);
    });
    ctx.stroke();
  }

  if (layers.laser && pose) {
    const laserList =
      lasers && lasers.length
        ? lasers
        : laser
          ? [{ scan: { ...laser, range_min: undefined, range_max: undefined }, color: '#ffeb3b', size: 0.05, alpha: 1 }]
          : [];
    for (const item of laserList) {
      const { scan, color, size, alpha } = item;
      const px = Math.max(1, size * scale * 0.35);
      ctx.fillStyle = color.includes('rgba')
        ? color
        : (() => {
            const h = color.replace('#', '');
            const full = h.length === 3 ? h.split('').map((c) => c + c).join('') : h;
            const n = Number.parseInt(full, 16);
            if (!Number.isFinite(n)) return `rgba(255,235,59,${alpha})`;
            return `rgba(${(n >> 16) & 255},${(n >> 8) & 255},${n & 255},${alpha})`;
          })();
      const rMin = scan.range_min ?? 0;
      const rMax = scan.range_max ?? Infinity;
      scan.ranges.forEach((r, i) => {
        if (!Number.isFinite(r) || r < rMin || r > rMax) return;
        const a =
          (scan.angle_min ?? 0) +
          i * (scan.angle_increment ?? 0) +
          (pose.yaw ?? 0);
        const [sx, sy] = toScreen(
          pose.x + r * Math.cos(a),
          pose.y + r * Math.sin(a),
        );
        ctx.fillRect(sx - px / 2, sy - px / 2, px, px);
      });
    }

    if (rangeOverlays?.length) {
      for (const item of rangeOverlays) {
        const yaw = pose.yaw ?? 0;
        const fov = item.field_of_view || 0.2;
        const r = item.range;
        ctx.strokeStyle = item.color.includes('rgba')
          ? item.color
          : (() => {
              const h = item.color.replace('#', '');
              const full = h.length === 3 ? h.split('').map((c) => c + c).join('') : h;
              const n = Number.parseInt(full, 16);
              if (!Number.isFinite(n)) return `rgba(128,203,196,${item.alpha})`;
              return `rgba(${(n >> 16) & 255},${(n >> 8) & 255},${n & 255},${item.alpha})`;
            })();
        ctx.fillStyle = ctx.strokeStyle;
        ctx.lineWidth = 1.5;
        const [ox, oy] = toScreen(pose.x, pose.y);
        ctx.beginPath();
        ctx.moveTo(ox, oy);
        const steps = 12;
        for (let i = 0; i <= steps; i++) {
          const a = yaw - fov / 2 + (fov * i) / steps;
          const [sx, sy] = toScreen(pose.x + r * Math.cos(a), pose.y + r * Math.sin(a));
          ctx.lineTo(sx, sy);
        }
        ctx.closePath();
        ctx.globalAlpha = Math.min(1, item.alpha * 0.35);
        ctx.fill();
        ctx.globalAlpha = 1;
        ctx.stroke();
      }
    }
  }

  if ((layers.pointcloud ?? true) && cloud?.points?.length) {
    const px = Math.max(1, cloud.size * scale * 0.4);
    let zMin = Infinity;
    let zMax = -Infinity;
    if (cloud.colorMode === 'height') {
      for (const p of cloud.points) {
        zMin = Math.min(zMin, p.z);
        zMax = Math.max(zMax, p.z);
      }
    }
    const zSpan = Math.max(1e-6, zMax - zMin);
    for (const p of cloud.points) {
      const [sx, sy] = toScreen(p.x, p.y);
      if (cloud.colorMode === 'height') {
        const t01 = (p.z - zMin) / zSpan;
        ctx.fillStyle = `rgba(${Math.round(255 * t01)},${Math.round(100 + 100 * (1 - t01))},${Math.round(255 * (1 - t01))},${cloud.alpha})`;
      } else {
        const inten = p.i ?? 0.7;
        ctx.fillStyle = `rgba(${Math.round(50 + 200 * inten)},${Math.round(150 * inten)},${Math.round(255 - 120 * inten)},${cloud.alpha})`;
      }
      ctx.fillRect(sx - px / 2, sy - px / 2, px, px);
    }
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
    drawPoseHandle(ctx, toScreen, scale, {
      x: goal.x,
      y: goal.y,
      yaw: goal.yaw ?? 0,
      label: 'G',
      ringM: 0.45,
    }, '#ff7043');
    if (pose) {
      const [sx, sy] = toScreen(goal.x, goal.y);
      ctx.strokeStyle = '#ff7043';
      ctx.lineWidth = 1.5;
      ctx.setLineDash([4, 4]);
      ctx.beginPath();
      const [rx, ry] = toScreen(pose.x, pose.y);
      ctx.moveTo(rx, ry);
      ctx.lineTo(sx, sy);
      ctx.stroke();
      ctx.setLineDash([]);
    }
  }

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

  if (measurePts.length) {
    ctx.strokeStyle = '#ffee58';
    ctx.fillStyle = '#ffee58';
    ctx.lineWidth = 1.5;

    // Confirmed solid segments
    if (measurePts.length >= 2) {
      ctx.setLineDash([]);
      ctx.beginPath();
      measurePts.forEach((p, i) => {
        const [sx, sy] = toScreen(p.x, p.y);
        if (i === 0) ctx.moveTo(sx, sy);
        else ctx.lineTo(sx, sy);
      });
      ctx.stroke();

      let total = 0;
      for (let i = 1; i < measurePts.length; i++) {
        const a = measurePts[i - 1];
        const b = measurePts[i];
        const seg = Math.hypot(b.x - a.x, b.y - a.y);
        total += seg;
        const [ax, ay] = toScreen(a.x, a.y);
        const [bx, by] = toScreen(b.x, b.y);
        const mx = (ax + bx) / 2;
        const my = (ay + by) / 2;
        const dx = bx - ax;
        const dy = by - ay;
        const len = Math.hypot(dx, dy) || 1;
        const ox = (-dy / len) * 10;
        const oy = (dx / len) * 10;
        ctx.font = '11px ui-sans-serif, system-ui, sans-serif';
        ctx.fillStyle = '#ffee58';
        ctx.strokeStyle = 'rgba(11,15,20,0.75)';
        ctx.lineWidth = 3;
        const label = `${seg.toFixed(2)} m`;
        ctx.strokeText(label, mx + ox, my + oy);
        ctx.fillText(label, mx + ox, my + oy);
        ctx.strokeStyle = '#ffee58';
        ctx.lineWidth = 1.5;
      }

      const last = measurePts[measurePts.length - 1];
      const [sx, sy] = toScreen(last.x, last.y);
      const totalLabel = `Σ ${total.toFixed(2)} m`;
      ctx.font = 'bold 12px ui-sans-serif, system-ui, sans-serif';
      ctx.strokeStyle = 'rgba(11,15,20,0.8)';
      ctx.lineWidth = 3;
      ctx.fillStyle = '#fff59d';
      ctx.strokeText(totalLabel, sx + 10, sy + 16);
      ctx.fillText(totalLabel, sx + 10, sy + 16);
    }

    // Dashed preview from last confirmed point → cursor (before next click)
    if (measurePreview && measurePts.length >= 1) {
      const last = measurePts[measurePts.length - 1];
      const [ax, ay] = toScreen(last.x, last.y);
      const [bx, by] = toScreen(measurePreview.x, measurePreview.y);
      const seg = Math.hypot(measurePreview.x - last.x, measurePreview.y - last.y);
      ctx.strokeStyle = '#ffee58';
      ctx.lineWidth = 1.5;
      ctx.setLineDash([6, 5]);
      ctx.beginPath();
      ctx.moveTo(ax, ay);
      ctx.lineTo(bx, by);
      ctx.stroke();
      ctx.setLineDash([]);
      // Preview distance (lighter)
      const mx = (ax + bx) / 2;
      const my = (ay + by) / 2;
      ctx.font = '11px ui-sans-serif, system-ui, sans-serif';
      ctx.strokeStyle = 'rgba(11,15,20,0.75)';
      ctx.lineWidth = 3;
      ctx.fillStyle = 'rgba(255, 245, 157, 0.85)';
      const previewLabel = `${seg.toFixed(2)} m`;
      ctx.strokeText(previewLabel, mx + 6, my - 6);
      ctx.fillText(previewLabel, mx + 6, my - 6);
    }

    measurePts.forEach((p) => {
      const [sx, sy] = toScreen(p.x, p.y);
      ctx.fillStyle = '#ffee58';
      ctx.fillRect(sx - 3, sy - 3, 6, 6);
      if (p.label) {
        ctx.font = '11px ui-sans-serif, system-ui, sans-serif';
        ctx.fillStyle = '#ffee58';
        ctx.fillText(p.label, sx + 6, sy - 6);
      }
    });
  }

  if (poseHandles.length) {
    // Connect AB with a light guide when two handles
    if (poseHandles.length === 2) {
      const [a, b] = poseHandles;
      const [ax, ay] = toScreen(a.x, a.y);
      const [bx, by] = toScreen(b.x, b.y);
      ctx.strokeStyle = 'rgba(105, 240, 174, 0.55)';
      ctx.lineWidth = 1.5;
      ctx.setLineDash([6, 4]);
      ctx.beginPath();
      ctx.moveTo(ax, ay);
      ctx.lineTo(bx, by);
      ctx.stroke();
      ctx.setLineDash([]);
    }
    poseHandles.forEach((p) => {
      drawPoseHandle(ctx, toScreen, scale, p, p.editing ? '#69f0ae' : '#80cbc4');
    });
  }

  if (routePts.length) {
    // Route polyline uses muted cyan; markers keep per-point colors.
    ctx.strokeStyle = 'rgba(79, 195, 247, 0.55)';
    ctx.lineWidth = 1.5;
    if (routePts.length > 1) {
      ctx.beginPath();
      routePts.forEach((p, i) => {
        const [sx, sy] = toScreen(p.x, p.y);
        if (i === 0) ctx.moveTo(sx, sy);
        else ctx.lineTo(sx, sy);
      });
      ctx.stroke();
    }
    routePts.forEach((p, i) => {
      const color = p.color ?? '#4fc3f7';
      const selected = !!p.selected;
      if (selected) {
        const [sx, sy] = toScreen(p.x, p.y);
        const ringPx = Math.max(16, 0.55 * scale);
        ctx.save();
        ctx.strokeStyle = '#ffee58';
        ctx.lineWidth = 3;
        ctx.globalAlpha = 0.9;
        ctx.beginPath();
        ctx.arc(sx, sy, ringPx + 5, 0, Math.PI * 2);
        ctx.stroke();
        ctx.restore();
      }
      drawPoseHandle(
        ctx,
        toScreen,
        scale,
        {
          x: p.x,
          y: p.y,
          yaw: p.yaw ?? 0,
          label: p.label ?? `#${i + 1}`,
          ringM: selected || p.editing ? 0.55 : 0.4,
          editing: !!p.editing || selected,
        },
        color,
      );
    });
  }
}
