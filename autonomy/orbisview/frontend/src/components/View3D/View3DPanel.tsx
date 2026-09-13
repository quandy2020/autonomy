import { useEffect, useMemo, useRef } from 'react';
import {
  createCameraController,
  type CameraController,
} from '@/renderer/view3d/cameraController';
import { createView3DScene, type View3DContext } from '@/renderer/view3d/createScene';
import { toThree } from '@/renderer/view3d/coords';
import { syncView3DScene } from '@/renderer/view3d/syncScene';
import type {
  OccupancyGridJson,
  Pose2D,
  RobotFootprintJson,
} from '@/renderer/map2d/types';
import type {
  View3DCloudPoint,
  View3DLaserScan,
  View3DSceneInput,
} from '@/renderer/view3d/types';
import { useDataStore } from '@/store/dataStore';
import { useLayerStore } from '@/store/layoutStore';
import { useView3DStore } from '@/store/view3dStore';
import { SCHEMAS, type StreamEnvelope } from '@/store/websocket/types';

function asPayload<T>(env: { payload?: unknown } | undefined): T | null {
  if (!env?.payload || typeof env.payload !== 'object') return null;
  return env.payload as T;
}

function pickMapAndCostmap(envelopes: Record<string, StreamEnvelope>) {
  const grids = Object.values(envelopes).filter((e) => e.schema === SCHEMAS.OccupancyGrid);
  const costmapEnv = grids.find((e) => /costmap/i.test(e.channel));
  const mapEnv =
    grids.find((e) => e !== costmapEnv && !/costmap/i.test(e.channel)) ??
    grids.find((e) => e !== costmapEnv);
  return {
    map: asPayload<OccupancyGridJson>(mapEnv),
    costmap: asPayload<OccupancyGridJson>(costmapEnv),
  };
}

export function View3DPanel() {
  const mountRef = useRef<HTMLDivElement>(null);
  const envelopes = useDataStore((s) => s.envelopes);
  const layers = useLayerStore();
  const followRobot = useLayerStore((s) => s.followRobot);
  const setFollowRobot = useLayerStore((s) => s.setFollowRobot);
  const cloudColor = useView3DStore((s) => s.cloudColor);
  const laserHeight = useView3DStore((s) => s.laserHeight);
  const mapOpacity = useView3DStore((s) => s.mapOpacity);
  const setCloudColor = useView3DStore((s) => s.setCloudColor);
  const setLaserHeight = useView3DStore((s) => s.setLaserHeight);
  const setMapOpacity = useView3DStore((s) => s.setMapOpacity);

  const sceneRef = useRef<View3DContext | null>(null);
  const camRef = useRef<CameraController | null>(null);

  useEffect(() => {
    const mount = mountRef.current;
    if (!mount) return;
    const ctx = createView3DScene(mount);
    const cam = createCameraController(ctx.camera);
    cam.setFollow(followRobot);
    sceneRef.current = ctx;
    camRef.current = cam;

    const el = ctx.renderer.domElement;
    el.addEventListener('pointerdown', cam.onPointerDown);
    window.addEventListener('pointerup', cam.onPointerUp);
    window.addEventListener('pointermove', cam.onPointerMove);
    el.addEventListener('wheel', cam.onWheel, { passive: false });

    const ro = new ResizeObserver((entries) => {
      const cr = entries[0]?.contentRect;
      if (!cr) return;
      ctx.setSize(cr.width, Math.max(cr.height, 240));
    });
    ro.observe(mount);

    let raf = 0;
    const tick = () => {
      ctx.renderer.render(ctx.scene, ctx.camera);
      raf = requestAnimationFrame(tick);
    };
    tick();

    return () => {
      cancelAnimationFrame(raf);
      ro.disconnect();
      el.removeEventListener('pointerdown', cam.onPointerDown);
      window.removeEventListener('pointerup', cam.onPointerUp);
      window.removeEventListener('pointermove', cam.onPointerMove);
      el.removeEventListener('wheel', cam.onWheel);
      ctx.dispose();
      sceneRef.current = null;
      camRef.current = null;
    };
    // followRobot applied in sync effect via setFollow
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  const input: View3DSceneInput = useMemo(() => {
    const pose = asPayload<Pose2D>(
      Object.values(envelopes).find((e) => e.schema === SCHEMAS.Pose2D),
    );
    const pathPayload = asPayload<{ poses: Pose2D[] }>(
      Object.values(envelopes).find((e) => e.schema === SCHEMAS.Path2D),
    );
    const nav = asPayload<{ has_goal?: boolean; goal?: { x: number; y: number } }>(
      Object.values(envelopes).find((e) => e.schema === SCHEMAS.Navigation),
    );
    const cloudPayload = asPayload<{ points: View3DCloudPoint[] }>(
      Object.values(envelopes).find((e) => e.schema === SCHEMAS.PointCloud2),
    );
    const footprint = asPayload<RobotFootprintJson>(
      Object.values(envelopes).find((e) => e.schema === SCHEMAS.RobotFootprint),
    );
    const laser = asPayload<View3DLaserScan>(
      Object.values(envelopes).find((e) => e.schema === SCHEMAS.LaserScan),
    );
    const { map, costmap } = pickMapAndCostmap(envelopes);
    const goal =
      nav?.goal && nav.has_goal !== false
        ? { x: nav.goal.x, y: nav.goal.y }
        : null;

    return {
      pose,
      path: pathPayload?.poses ?? null,
      goal,
      cloud: cloudPayload?.points ?? null,
      footprint,
      map,
      costmap,
      laser,
      layers: {
        grid: layers.grid,
        robot: layers.robot,
        path: layers.path,
        pointcloud: layers.pointcloud,
        footprint: layers.footprint,
        map: layers.map,
        costmap: layers.costmap,
        laser: layers.laser,
      },
      opts: { cloudColor, laserHeight, mapOpacity },
      followRobot,
    };
  }, [envelopes, layers, cloudColor, laserHeight, mapOpacity, followRobot]);

  useEffect(() => {
    const ctx = sceneRef.current;
    const cam = camRef.current;
    if (!ctx || !cam) return;
    cam.setFollow(input.followRobot);
    if (input.followRobot && input.pose) {
      const t = toThree(input.pose.x, input.pose.y, 0);
      cam.setTarget(t.x, 0, t.z);
    }
    cam.update();
    syncView3DScene(ctx, input);
  }, [input]);

  const cloudStale = Object.values(envelopes).find((e) => e.schema === SCHEMAS.PointCloud2)
    ?.stale;

  return (
    <div className="panel view3d-panel">
      <div className="view3d-toolbar">
        <div className="view3d-toolbar-modes">
          <button
            type="button"
            className={followRobot ? 'tab active' : 'tab'}
            onClick={() => setFollowRobot(true)}
          >
            Follow
          </button>
          <button
            type="button"
            className={!followRobot ? 'tab active' : 'tab'}
            onClick={() => setFollowRobot(false)}
          >
            Free
          </button>
          <button type="button" className="tab" onClick={() => camRef.current?.reset()}>
            Reset
          </button>
        </div>
        <div className="view3d-toolbar-opts">
          <label className="view3d-opt" title="Point cloud color mode">
            <span className="view3d-opt-label">Cloud</span>
            <select
              value={cloudColor}
              onChange={(e) => setCloudColor(e.target.value as 'height' | 'intensity')}
            >
              <option value="intensity">intensity</option>
              <option value="height">height</option>
            </select>
          </label>
          <label className="view3d-opt" title="Laser height (m)">
            <span className="view3d-opt-label">Laser</span>
            <input
              type="range"
              min={0}
              max={1}
              step={0.05}
              value={laserHeight}
              onChange={(e) => setLaserHeight(Number(e.target.value))}
            />
            <span className="view3d-opt-val">{laserHeight.toFixed(2)}</span>
          </label>
          <label className="view3d-opt" title="Map / costmap opacity">
            <span className="view3d-opt-label">Map</span>
            <input
              type="range"
              min={0.1}
              max={1}
              step={0.05}
              value={mapOpacity}
              onChange={(e) => setMapOpacity(Number(e.target.value))}
            />
            <span className="view3d-opt-val">{mapOpacity.toFixed(2)}</span>
          </label>
        </div>
      </div>
      {cloudStale ? <div className="stale-badge">pointcloud stale</div> : null}
      <div className="view3d-host" ref={mountRef} />
      <p className="hint">drag orbit · shift-drag pan (free) · wheel zoom · layers shared with Map2D</p>
    </div>
  );
}
