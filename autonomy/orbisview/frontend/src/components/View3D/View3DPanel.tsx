import { MapFloatToolbar } from '@/components/Map/MapFloatToolbar';
import { MapInstrumentCluster } from '@/components/Map/MapInstrumentCluster';
import {
  createCameraController,
  type CameraController,
} from '@/renderer/view3d/cameraController';
import { createView3DScene, type View3DContext } from '@/renderer/view3d/createScene';
import { fromThree, toThree } from '@/renderer/view3d/coords';
import { syncView3DScene } from '@/renderer/view3d/syncScene';
import { updateToolOverlay } from '@/renderer/view3d/layers/toolOverlay';
import type {
  OccupancyGridJson,
  Pose2D,
  RobotFootprintJson,
} from '@/renderer/map2d/types';
import type {
  View3DLaserScan,
  View3DSceneInput,
} from '@/renderer/view3d/types';
import { useDataStore } from '@/store/dataStore';
import { useLayerStore, useLayoutStore } from '@/store/layoutStore';
import { useView3DStore } from '@/store/view3dStore';
import { useWaypointStore, waypointColor } from '@/store/waypointStore';
import { useMapViewStore } from '@/store/mapViewStore';
import { SCHEMAS } from '@/store/websocket/types';
import { wsClient } from '@/store/websocket/client';
import { useEffect, useMemo, useRef, useState } from 'react';
import * as THREE from 'three';
import {
  asPayload,
  effectiveMapLayers,
  pickDisplayEnvelope,
} from '@/components/Channels/mapDisplayBinding';
import {
  hexToThree,
  resolveCloudOverlay,
  resolveLaserOverlays,
  resolvePathStyle,
} from '@/components/Channels/sensorDisplay';
import { useDisplayStore } from '@/store/displayStore';
import { formatPickPose, pickPoseStatus } from '@/utils/poseMath';

function findWpId(obj: THREE.Object3D | null): string | null {
  let cur: THREE.Object3D | null = obj;
  while (cur) {
    if (typeof cur.userData?.wpId === 'string') return cur.userData.wpId as string;
    cur = cur.parent;
  }
  return null;
}

function yawDeg(yaw: number) {
  return (((yaw * 180) / Math.PI) % 360 + 360) % 360;
}

function pathLength(pts: { x: number; y: number }[]) {
  let dist = 0;
  for (let i = 1; i < pts.length; i++) {
    dist += Math.hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
  }
  return dist;
}

type PoseDrag = {
  tool: 'nav' | 'pick';
  x: number;
  y: number;
  yaw: number;
  moved: boolean;
  label: string;
};

type SketchPt = { x: number; y: number; yaw?: number; label?: string };

export function View3DPanel({ active = true }: { active?: boolean }) {
  const mountRef = useRef<HTMLDivElement>(null);
  const envelopes = useDataStore((s) => s.envelopes);
  const displays = useDisplayStore((s) => s.displays);
  const layers = useLayerStore();
  const followRobot = useLayerStore((s) => s.followRobot);
  const setFollowRobot = useLayerStore((s) => s.setFollowRobot);
  const cloudColor = useView3DStore((s) => s.cloudColor);
  const laserHeight = useView3DStore((s) => s.laserHeight);
  const mapOpacity = useView3DStore((s) => s.mapOpacity);
  const waypoints = useWaypointStore((s) => s.waypoints);
  const selectedId = useWaypointStore((s) => s.selectedId);
  const mapTool = useMapViewStore((s) => s.tool);
  const setStatusMsg = useMapViewStore((s) => s.setStatusMsg);

  const [localGoal, setLocalGoal] = useState<Pose2D | null>(null);
  const [sketchPts, setSketchPts] = useState<SketchPt[]>([]);
  const [measurePreview, setMeasurePreview] = useState<{ x: number; y: number } | null>(null);
  const [sceneReady, setSceneReady] = useState(false);

  const sceneRef = useRef<View3DContext | null>(null);
  const camRef = useRef<CameraController | null>(null);
  const wpDragRef = useRef<{ id: string } | null>(null);
  const poseDragRef = useRef<PoseDrag | null>(null);
  const pickDownRef = useRef<{ x: number; y: number; sx: number; sy: number } | null>(null);
  const orbitFromToolRef = useRef(false);
  const sketchRef = useRef(sketchPts);
  sketchRef.current = sketchPts;
  const measurePreviewRef = useRef(measurePreview);
  measurePreviewRef.current = measurePreview;
  const activeRef = useRef(active);
  activeRef.current = active;
  const inputRef = useRef<View3DSceneInput | null>(null);

  const raycaster = useMemo(() => new THREE.Raycaster(), []);
  const pointer = useMemo(() => new THREE.Vector2(), []);
  const groundPlane = useMemo(() => new THREE.Plane(new THREE.Vector3(0, 1, 0), 0), []);

  useEffect(() => {
    setSketchPts([]);
    setMeasurePreview(null);
    poseDragRef.current = null;
    pickDownRef.current = null;
    if (mapTool === 'pan') {
      setFollowRobot(false);
      setStatusMsg('拖动视图：左键旋转，Shift+左键平移，滚轮缩放');
    }
  }, [mapTool, setFollowRobot, setStatusMsg]);

  useEffect(() => {
    const mount = mountRef.current;
    if (!mount) return;
    const ctx = createView3DScene(mount);
    const cam = createCameraController(ctx.camera);
    cam.setFollow(followRobot);
    sceneRef.current = ctx;
    camRef.current = cam;

    const el = ctx.renderer.domElement;
    el.style.cursor = 'crosshair';

    const ndcFromEvent = (e: { clientX: number; clientY: number }) => {
      const rect = el.getBoundingClientRect();
      pointer.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
      pointer.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
    };

    const hitWaypoint = (e: { clientX: number; clientY: number }) => {
      ndcFromEvent(e);
      raycaster.setFromCamera(pointer, ctx.camera);
      const hits = raycaster.intersectObjects(ctx.waypoints.children, true);
      return findWpId(hits[0]?.object ?? null);
    };

    const groundHit = (e: { clientX: number; clientY: number }) => {
      ndcFromEvent(e);
      raycaster.setFromCamera(pointer, ctx.camera);
      const target = new THREE.Vector3();
      if (!raycaster.ray.intersectPlane(groundPlane, target)) return null;
      return fromThree(target.x, target.y, target.z);
    };

    const finishPoseDrag = (drag: PoseDrag) => {
      const yaw = drag.moved ? drag.yaw : 0;
      if (drag.tool === 'pick') {
        const text = formatPickPose(drag.x, drag.y, yaw);
        void navigator.clipboard?.writeText(text);
        setSketchPts([]);
        setStatusMsg(pickPoseStatus(drag.x, drag.y, yaw));
        return;
      }
      const n = useWaypointStore.getState().waypoints.length + 1;
      useWaypointStore.getState().add(drag.x, drag.y, yaw, `#${n}`);
      setSketchPts([]);
      const list = useWaypointStore.getState().waypoints;
      if (list.length === 1) {
        setLocalGoal({ x: drag.x, y: drag.y, yaw });
        if (useDataStore.getState().connected) {
          wsClient.send({ op: 'set_goal', x: drag.x, y: drag.y, yaw });
        }
        setStatusMsg(
          `目标点 (${drag.x.toFixed(2)}, ${drag.y.toFixed(2)}) yaw ${yawDeg(yaw).toFixed(0)}° · 已发送`,
        );
        return;
      }
      setStatusMsg(`导航点 #${list.length} · 共 ${list.length} 点 · 点发送下发路线`);
    };

    const finishMeasureAt = (end: { x: number; y: number } | null) => {
      const start = sketchRef.current[0];
      if (start && end && sketchRef.current.length === 1) {
        const pts = [start, end];
        sketchRef.current = pts;
        setSketchPts(pts);
        setMeasurePreview(null);
        const dist = pathLength(pts);
        setStatusMsg(
          `测距 ${dist.toFixed(2)} m · (${start.x.toFixed(2)}, ${start.y.toFixed(2)}) → (${end.x.toFixed(2)}, ${end.y.toFixed(2)}) · 结果保留`,
        );
        return;
      }
      if (sketchRef.current.length >= 2) {
        setStatusMsg(
          `测距 ${pathLength(sketchRef.current).toFixed(2)} m · 结果保留 · 左键可重新开始`,
        );
        return;
      }
      setStatusMsg('测距：请先左键确定起点');
    };

    const onPointerDown = (e: PointerEvent) => {
      const tool = useMapViewStore.getState().tool;

      // Measure: RMB places end — do not orbit / suppress browser menu.
      if (tool === 'measure' && e.button === 2) {
        e.preventDefault();
        return;
      }

      // Pan / orbit tool: always drive the camera (Shift = pan target).
      if (tool === 'pan' || e.button === 1 || e.button === 2) {
        e.preventDefault();
        setFollowRobot(false);
        cam.onPointerDown(e);
        el.style.cursor = 'grabbing';
        return;
      }

      if (e.button !== 0) {
        cam.onPointerDown(e);
        return;
      }

      const id = hitWaypoint(e);
      if (id && tool !== 'measure' && tool !== 'pick') {
        useWaypointStore.getState().select(id);
        wpDragRef.current = { id };
        setFollowRobot(false);
        setStatusMsg('3D 选中航点 · 拖动改位置 · 双击打开 Waypoints');
        return;
      }

      const g = groundHit(e);
      if (!g) {
        cam.onPointerDown(e);
        return;
      }

      if (tool === 'measure') {
        pickDownRef.current = { ...g, sx: e.clientX, sy: e.clientY };
        orbitFromToolRef.current = false;
        return;
      }

      if (tool === 'nav' || tool === 'pick') {
        setFollowRobot(false);
        const label =
          tool === 'pick'
            ? 'Pick'
            : `#${useWaypointStore.getState().waypoints.length + 1}`;
        poseDragRef.current = {
          tool,
          x: g.x,
          y: g.y,
          yaw: 0,
          moved: false,
          label,
        };
        setSketchPts([{ x: g.x, y: g.y, yaw: 0, label }]);
        setStatusMsg(
          tool === 'pick'
            ? `取点 (${g.x.toFixed(2)}, ${g.y.toFixed(2)}) · 拖动设朝向 · 松手复制`
            : `${label} · 拖动设朝向 · 松手确认`,
        );
        return;
      }

      cam.onPointerDown(e);
    };
    const onDoubleClick = (e: MouseEvent) => {
      const id = hitWaypoint(e);
      if (!id) return;
      e.preventDefault();
      useWaypointStore.getState().select(id);
      wpDragRef.current = null;
      useLayoutStore.getState().ensurePanel('waypoints');
      setStatusMsg('已打开 Waypoints · 可在列表中编辑');
    };

    const onPointerMove = (e: PointerEvent) => {
      const wpDrag = wpDragRef.current;
      if (wpDrag) {
        const g = groundHit(e);
        if (g) {
          useWaypointStore.getState().update(wpDrag.id, { x: g.x, y: g.y });
          setStatusMsg(`3D 移动 (${g.x.toFixed(2)}, ${g.y.toFixed(2)})`);
        }
        return;
      }

      const poseDrag = poseDragRef.current;
      if (poseDrag) {
        const g = groundHit(e);
        if (!g) return;
        const dist = Math.hypot(g.x - poseDrag.x, g.y - poseDrag.y);
        if (dist > 0.08) {
          poseDrag.moved = true;
          poseDrag.yaw = Math.atan2(g.y - poseDrag.y, g.x - poseDrag.x);
        }
        setSketchPts([
          {
            x: poseDrag.x,
            y: poseDrag.y,
            yaw: poseDrag.yaw,
            label: poseDrag.label,
          },
        ]);
        setStatusMsg(
          poseDrag.tool === 'pick'
            ? `取点 (${poseDrag.x.toFixed(2)}, ${poseDrag.y.toFixed(2)}) · 朝向 ${yawDeg(poseDrag.yaw).toFixed(0)}°`
            : `${poseDrag.label} · 朝向 ${yawDeg(poseDrag.yaw).toFixed(0)}°`,
        );
        return;
      }

      const tool = useMapViewStore.getState().tool;
      const pending = pickDownRef.current;
      if (pending && tool === 'measure' && !orbitFromToolRef.current) {
        const screenDist = Math.hypot(e.clientX - pending.sx, e.clientY - pending.sy);
        if (screenDist > 6) {
          orbitFromToolRef.current = true;
          cam.onPointerDown(e);
        }
      }

      if (tool === 'measure' && sketchRef.current.length === 1 && !pickDownRef.current) {
        const g = groundHit(e);
        if (g) setMeasurePreview(g);
        return;
      }

      cam.onPointerMove(e);
    };
    const onPointerUp = (e: PointerEvent) => {
      const tool = useMapViewStore.getState().tool;
      el.style.cursor = tool === 'pan' ? 'grab' : 'crosshair';

      if (tool === 'measure' && e.button === 2) {
        const end = measurePreviewRef.current ?? groundHit(e);
        finishMeasureAt(end);
        return;
      }

      if (wpDragRef.current) {
        const id = wpDragRef.current.id;
        wpDragRef.current = null;
        const wp = useWaypointStore.getState().waypoints.find((w) => w.id === id);
        if (wp) {
          const list = useWaypointStore.getState().waypoints;
          if (list.length === 1 && useDataStore.getState().connected) {
            setLocalGoal({ x: wp.x, y: wp.y, yaw: wp.yaw ?? 0 });
            wsClient.send({ op: 'set_goal', x: wp.x, y: wp.y, yaw: wp.yaw ?? 0 });
            setStatusMsg(
              `已更新目标 (${wp.x.toFixed(2)}, ${wp.y.toFixed(2)})`,
            );
          } else {
            setStatusMsg(
              `3D 已更新 ${wp.label ?? '#'} (${wp.x.toFixed(2)}, ${wp.y.toFixed(2)})`,
            );
          }
        }
        return;
      }

      if (poseDragRef.current) {
        const drag = poseDragRef.current;
        poseDragRef.current = null;
        finishPoseDrag(drag);
        return;
      }

      const down = pickDownRef.current;
      const orbited = orbitFromToolRef.current;
      pickDownRef.current = null;
      orbitFromToolRef.current = false;

      if (orbited) {
        cam.onPointerUp();
        return;
      }

      if (down && tool === 'measure') {
        // Left click only sets / resets the start point.
        setSketchPts([{ x: down.x, y: down.y }]);
        setMeasurePreview(null);
        setStatusMsg(
          `测距起点 (${down.x.toFixed(2)}, ${down.y.toFixed(2)}) · 移动预览 · 右击落终点`,
        );
        return;
      }

      cam.onPointerUp();
    };
    const onWheel = (e: WheelEvent) => {
      const poseDrag = poseDragRef.current;
      if (poseDrag) {
        e.preventDefault();
        poseDrag.moved = true;
        poseDrag.yaw += e.deltaY * 0.004;
        setSketchPts([
          {
            x: poseDrag.x,
            y: poseDrag.y,
            yaw: poseDrag.yaw,
            label: poseDrag.label,
          },
        ]);
        setStatusMsg(`朝向 ${yawDeg(poseDrag.yaw).toFixed(0)}°`);
        return;
      }

      const sel = useWaypointStore.getState().selectedId;
      if (e.shiftKey && sel) {
        e.preventDefault();
        const wp = useWaypointStore.getState().waypoints.find((w) => w.id === sel);
        if (!wp) return;
        const yaw = (wp.yaw ?? 0) + e.deltaY * 0.004;
        useWaypointStore.getState().update(sel, { yaw });
        setStatusMsg(`朝向 ${yawDeg(yaw).toFixed(0)}°`);
        return;
      }
      cam.onWheel(e);
    };

    const onContextMenu = (e: MouseEvent) => {
      e.preventDefault();
      const tool = useMapViewStore.getState().tool;
      if (poseDragRef.current) {
        poseDragRef.current = null;
        setSketchPts([]);
        setStatusMsg('已取消落点');
        return;
      }
      if (tool === 'measure') {
        // Prefer pointerup(button=2); keep as fallback if contextmenu arrives alone.
        const end = measurePreviewRef.current ?? groundHit(e);
        finishMeasureAt(end);
        return;
      }
      if (tool === 'nav') {
        setSketchPts([]);
        setStatusMsg('导航：落点设朝向；1 点发目标，多点发路线');
      }
    };

    el.addEventListener('pointerdown', onPointerDown);
    el.addEventListener('dblclick', onDoubleClick);
    el.addEventListener('contextmenu', onContextMenu);
    window.addEventListener('pointerup', onPointerUp);
    window.addEventListener('pointermove', onPointerMove);
    el.addEventListener('wheel', onWheel, { passive: false });

    const ro = new ResizeObserver((entries) => {
      const cr = entries[0]?.contentRect;
      if (!cr || cr.width < 2 || cr.height < 2) return;
      ctx.setSize(cr.width, cr.height);
    });
    ro.observe(mount);

    const fitSize = () => {
      const w = mount.clientWidth;
      const h = mount.clientHeight;
      if (w >= 2 && h >= 2) ctx.setSize(w, h);
    };
    fitSize();
    requestAnimationFrame(fitSize);

    let raf = 0;
    const tick = () => {
      if (activeRef.current) {
        ctx.renderer.render(ctx.scene, ctx.camera);
      }
      raf = requestAnimationFrame(tick);
    };
    tick();
    setSceneReady(true);

    return () => {
      cancelAnimationFrame(raf);
      ro.disconnect();
      el.removeEventListener('pointerdown', onPointerDown);
      el.removeEventListener('dblclick', onDoubleClick);
      el.removeEventListener('contextmenu', onContextMenu);
      window.removeEventListener('pointerup', onPointerUp);
      window.removeEventListener('pointermove', onPointerMove);
      el.removeEventListener('wheel', onWheel);
      setSceneReady(false);
      ctx.dispose();
      sceneRef.current = null;
      camRef.current = null;
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  const nav = useMemo(() => {
    return asPayload<{ goal?: Pose2D; has_goal?: boolean }>(
      pickDisplayEnvelope(envelopes, displays, 'navigation'),
    );
  }, [envelopes, displays]);

  const goal =
    localGoal ??
    (nav?.has_goal !== false && nav?.goal
      ? { x: nav.goal.x, y: nav.goal.y, yaw: nav.goal.yaw }
      : null);

  const input = useMemo((): View3DSceneInput => {
    const pose = asPayload<Pose2D>(pickDisplayEnvelope(envelopes, displays, 'pose'));
    const pathPayload = asPayload<{ poses: Pose2D[] }>(
      pickDisplayEnvelope(envelopes, displays, 'path'),
    );
    const map = asPayload<OccupancyGridJson>(
      pickDisplayEnvelope(envelopes, displays, 'map'),
    );
    const costmap = asPayload<OccupancyGridJson>(
      pickDisplayEnvelope(envelopes, displays, 'costmap'),
    );
    const footprint = asPayload<RobotFootprintJson>(
      pickDisplayEnvelope(envelopes, displays, 'footprint'),
    );
    const laserOverlays = resolveLaserOverlays(envelopes, displays);
    const laser =
      laserOverlays[0]?.scan ??
      asPayload<View3DLaserScan>(pickDisplayEnvelope(envelopes, displays, 'laser'));
    const cloudOverlay = resolveCloudOverlay(envelopes, displays);
    const pathStyle = resolvePathStyle(displays);
    const paintLayers = effectiveMapLayers(layers, displays);
    const laserStyle = laserOverlays[0]?.style;
    const cloudStyle = cloudOverlay?.style;
    const cloudMode =
      cloudStyle?.colorMode === 'height' ? 'height' : cloudColor;

    return {
      pose,
      path: pathPayload?.poses ?? null,
      goal: goal ? { x: goal.x, y: goal.y } : null,
      waypoints: waypoints.map((w, i) => ({
        id: w.id,
        x: w.x,
        y: w.y,
        yaw: w.yaw ?? 0,
        label: w.label,
        color: waypointColor(i),
        selected: w.id === selectedId,
      })),
      map,
      costmap,
      footprint,
      laser,
      cloud: cloudOverlay?.points ?? null,
      layers: {
        grid: paintLayers.grid,
        map: paintLayers.map,
        costmap: paintLayers.costmap,
        path: paintLayers.path,
        robot: paintLayers.robot,
        footprint: paintLayers.footprint,
        laser: paintLayers.laser,
        pointcloud: paintLayers.pointcloud,
      },
      opts: {
        cloudColor: cloudMode,
        cloudSize: cloudStyle?.size,
        laserHeight,
        laserColor: laserStyle ? hexToThree(laserStyle.color) : undefined,
        laserSize: laserStyle?.size,
        mapOpacity,
        pathColor: hexToThree(pathStyle.color),
        pathOpacity: pathStyle.alpha,
        pathLineWidth: pathStyle.lineWidth,
      },
      followRobot,
    };
  }, [
    envelopes,
    displays,
    layers,
    cloudColor,
    laserHeight,
    mapOpacity,
    followRobot,
    waypoints,
    selectedId,
    goal,
  ]);
  inputRef.current = input;

  useEffect(() => {
    const ctx = sceneRef.current;
    const cam = camRef.current;
    if (!sceneReady || !ctx || !cam) return;
    cam.setFollow(followRobot);
    if (input.pose) {
      const t = toThree(input.pose.x, input.pose.y, 0);
      cam.setTarget(t.x, t.y, t.z);
    }
    cam.update();
    syncView3DScene(ctx, input);
  }, [input, followRobot, sceneReady]);

  /** When switching into 3D, remeasure the host (was display:none) and resync. */
  useEffect(() => {
    if (!active || !sceneReady) return;
    const ctx = sceneRef.current;
    const cam = camRef.current;
    const mount = mountRef.current;
    if (!ctx || !cam || !mount) return;
    const apply = () => {
      const w = mount.clientWidth;
      const h = mount.clientHeight;
      if (w >= 2 && h >= 2) ctx.setSize(w, h);
      const latest = inputRef.current;
      if (latest) {
        cam.setFollow(latest.followRobot);
        if (latest.pose) {
          const t = toThree(latest.pose.x, latest.pose.y, 0);
          cam.setTarget(t.x, t.y, t.z);
        }
        cam.update();
        syncView3DScene(ctx, latest);
      }
      ctx.renderer.render(ctx.scene, ctx.camera);
    };
    apply();
    let cancelled = false;
    const id = requestAnimationFrame(() => {
      requestAnimationFrame(() => {
        if (!cancelled) apply();
      });
    });
    return () => {
      cancelled = true;
      cancelAnimationFrame(id);
    };
  }, [active, sceneReady]);

  useEffect(() => {
    const ctx = sceneRef.current;
    if (!ctx) return;
    const isMeasure = mapTool === 'measure';
    const isPoseTool = mapTool === 'nav' || mapTool === 'pick';
    updateToolOverlay(ctx, {
      points: isMeasure || isPoseTool ? sketchPts : [],
      preview: isMeasure ? measurePreview : null,
      connect: isMeasure,
      dashedPreview: true,
    });
  }, [sketchPts, measurePreview, mapTool, sceneReady]);

  useEffect(() => {
    const el = sceneRef.current?.renderer.domElement;
    if (!el) return;
    el.style.cursor = mapTool === 'pan' ? 'grab' : 'crosshair';
  }, [mapTool, sceneReady]);

  const cloudStale = Object.values(envelopes).find((e) => e.schema === SCHEMAS.PointCloud2)
    ?.stale;

  const goalLabel = goal
    ? `goal (${goal.x.toFixed(2)}, ${goal.y.toFixed(2)}, ${yawDeg(goal.yaw ?? 0).toFixed(0)}°)`
    : undefined;

  const clearGoal = () => {
    setLocalGoal(null);
    setSketchPts([]);
    wsClient.send({ op: 'clear_goal' });
    setStatusMsg('已清除目标');
  };

  return (
    <div className="map-viewport view3d-panel">
      {cloudStale ? <div className="stale-badge map-float-badge">pointcloud stale</div> : null}
      <MapInstrumentCluster />
      <MapFloatToolbar
        measureActive={mapTool === 'measure' && sketchPts.length > 0}
        onClearMeasure={() => {
          setSketchPts([]);
          setMeasurePreview(null);
          setStatusMsg('测距：左键起点，移动预览，右击落终点');
        }}
        goalLabel={goalLabel}
        onClearGoal={clearGoal}
        onZoomIn={() => {
          camRef.current?.zoomIn();
          setFollowRobot(false);
          setStatusMsg('已放大');
        }}
        onZoomOut={() => {
          camRef.current?.zoomOut();
          setFollowRobot(false);
          setStatusMsg('已缩小');
        }}
        onFit={() => {
          camRef.current?.fit();
          if (input.pose) {
            const t = toThree(input.pose.x, input.pose.y, 0);
            camRef.current?.setTarget(t.x, t.y, t.z);
          }
          setFollowRobot(false);
          setStatusMsg('自适应视角');
        }}
      />
      <div className="view3d-host" ref={mountRef} />
    </div>
  );
}
