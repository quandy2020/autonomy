import { useEffect, useMemo, useRef, useState } from 'react';
import { DEFAULT_FOOTPRINT } from '@/config/parameters';
import { paintMap2DScene } from '@/renderer';
import type {
  OccupancyGridJson,
  Pose2D,
  RobotFootprintJson,
} from '@/renderer/map2d/types';
import { useDataStore } from '@/store/dataStore';
import { useLayerStore, useLayoutStore } from '@/store/layoutStore';
import { useWaypointStore, waypointColor } from '@/store/waypointStore';
import { useMapViewStore } from '@/store/mapViewStore';
import { SCHEMAS } from '@/store/websocket/types';
import { wsClient } from '@/store/websocket/client';
import { MapFloatToolbar } from '@/components/Map/MapFloatToolbar';
import { MapInstrumentCluster } from '@/components/Map/MapInstrumentCluster';
import { MapMappingHud } from '@/components/Map/MapMappingHud';
import { MapFloorBar } from '@/components/Map/MapFloorBar';
import { useStaticSlamStore } from '@/store/staticSlamStore';
import { useIndoorMapStore } from '@/store/indoorMapStore';
import { useAnnotationStore } from '@/store/annotationStore';
import { hitTestPoi } from '@/renderer/map2d/annotations';
import {
  sharedStaticSlamCanvasCache,
  staticSlamCorners,
  type StaticSlamCanvasHandle,
} from '@/renderer/map2d/staticSlam';
import {
  asPayload,
  effectiveMapLayers,
  pickDisplayEnvelope,
  pickDisplayEnvelopes,
} from '@/components/Channels/mapDisplayBinding';
import {
  hexToRgba,
  resolveCloudOverlay,
  resolveLaserOverlays,
  resolvePathStyle,
  resolveRangeOverlays,
} from '@/components/Channels/sensorDisplay';
import { useDisplayStore } from '@/store/displayStore';
import { useTfBufferStore } from '@/store/tfBufferStore';
import { formatPickPose, pickPoseStatus } from '@/utils/poseMath';
import { mergeTfTransforms } from '@/renderer/map2d/tfCompose';

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

interface TfTree {
  transforms: {
    parent: string;
    child: string;
    x: number;
    y: number;
    yaw?: number;
  }[];
}

const DEFAULT_SCALE = 40;
const MIN_SCALE = 8;
const MAX_SCALE = 160;

export function Map2DPanel() {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const hostRef = useRef<HTMLDivElement>(null);
  const envelopes = useDataStore((s) => s.envelopes);
  const connected = useDataStore((s) => s.connected);
  const displays = useDisplayStore((s) => s.displays);
  const layers = useLayerStore();
  const followRobot = useLayerStore((s) => s.followRobot);
  const staticBasemap = useStaticSlamStore((s) => s.basemap);
  const semanticZones = useIndoorMapStore((s) => s.zones);
  const annPois = useAnnotationStore((s) => s.pois);
  const annShapes = useAnnotationStore((s) => s.shapes);
  const annDraft = useAnnotationStore((s) => s.draft);
  const annDraftPreview = useAnnotationStore((s) => s.draftPreview);
  const annSelectedId = useAnnotationStore((s) => s.selectedId);
  const [basemapHandle, setBasemapHandle] = useState<StaticSlamCanvasHandle | null>(null);
  const waypoints = useWaypointStore((s) => s.waypoints);
  const selectedId = useWaypointStore((s) => s.selectedId);
  const addWaypoint = useWaypointStore((s) => s.add);
  const updateWaypoint = useWaypointStore((s) => s.update);
  const selectWaypoint = useWaypointStore((s) => s.select);
  const mapTool = useMapViewStore((s) => s.tool);
  const setStatusMsg = useMapViewStore((s) => s.setStatusMsg);
  const [localGoal, setLocalGoal] = useState<Pose2D | null>(null);
  const [viewOffset, setViewOffset] = useState({ x: 0, y: 0 });
  const [scale, setScale] = useState(DEFAULT_SCALE);
  const [canvasSize, setCanvasSize] = useState({ w: 640, h: 480 });
  const [measurePreview, setMeasurePreview] = useState<{ x: number; y: number } | null>(
    null,
  );

  const [sketchPts, setSketchPts] = useState<
    {
      x: number;
      y: number;
      yaw?: number;
      label?: string;
      ringM?: number;
      editing?: boolean;
    }[]
  >([]);
  const sketchRef = useRef(sketchPts);
  sketchRef.current = sketchPts;
  const measurePreviewRef = useRef(measurePreview);
  measurePreviewRef.current = measurePreview;

  type PoseDrag = {
    tool: 'nav' | 'pick';
    x: number;
    y: number;
    yaw: number;
    ringM: number;
    moved: boolean;
    label: string;
  };
  /** Drag an existing waypoint: move (near center) or set yaw (pull outward). */
  type WpEditDrag = {
    id: string;
    originX: number;
    originY: number;
    yaw: number;
    mode: 'move' | 'yaw';
    moved: boolean;
  };
  const poseDragRef = useRef<PoseDrag | null>(null);
  const wpEditRef = useRef<WpEditDrag | null>(null);
  const viewPanRef = useRef<{ lastX: number; lastY: number } | null>(null);
  const poiDragRef = useRef<{ id: string } | null>(null);
  const spaceHeldRef = useRef(false);

  useEffect(() => {
    setSketchPts([]);
    setMeasurePreview(null);
    poseDragRef.current = null;
    wpEditRef.current = null;
    viewPanRef.current = null;
    poiDragRef.current = null;
    if (mapTool !== 'draw') {
      useAnnotationStore.getState().cancelDraft();
    }
    if (mapTool === 'pan') {
      useLayerStore.getState().setFollowRobot(false);
      setStatusMsg('拖动视图：左键任意方向平移，中键/右键/空格+左键亦可，滚轮缩放');
    }
  }, [mapTool, setStatusMsg]);

  useEffect(() => {
    const isTypingTarget = (t: EventTarget | null) =>
      t instanceof HTMLInputElement ||
      t instanceof HTMLTextAreaElement ||
      (t instanceof HTMLElement && t.isContentEditable);

    const onKeyDown = (e: KeyboardEvent) => {
      if (isTypingTarget(e.target)) return;
      if (e.code === 'Space') {
        e.preventDefault();
        spaceHeldRef.current = true;
        return;
      }
      if (e.key === 'Escape') {
        useAnnotationStore.getState().cancelDraft();
        return;
      }
      if (e.key === 'Backspace' && mapTool === 'draw') {
        e.preventDefault();
        useAnnotationStore.getState().undoDraftPoint();
        return;
      }
      if ((e.key === 'Delete' || e.key === 'Backspace') && mapTool === 'poi') {
        const id = useAnnotationStore.getState().selectedId;
        if (id) {
          e.preventDefault();
          useAnnotationStore.getState().removePoi(id);
        }
      }
    };
    const onKeyUp = (e: KeyboardEvent) => {
      if (e.code === 'Space') spaceHeldRef.current = false;
    };
    window.addEventListener('keydown', onKeyDown);
    window.addEventListener('keyup', onKeyUp);
    return () => {
      window.removeEventListener('keydown', onKeyDown);
      window.removeEventListener('keyup', onKeyUp);
      spaceHeldRef.current = false;
    };
  }, [mapTool]);
  const pose = useMemo(() => {
    return asPayload<Pose2D>(pickDisplayEnvelope(envelopes, displays, 'pose'));
  }, [envelopes, displays]);

  const path = useMemo(() => {
    return asPayload<Path2D>(pickDisplayEnvelope(envelopes, displays, 'path'));
  }, [envelopes, displays]);

  const map = useMemo(() => {
    return asPayload<OccupancyGridJson>(pickDisplayEnvelope(envelopes, displays, 'map'));
  }, [envelopes, displays]);

  const costmap = useMemo(() => {
    return asPayload<OccupancyGridJson>(pickDisplayEnvelope(envelopes, displays, 'costmap'));
  }, [envelopes, displays]);

  const laser = useMemo(() => {
    return asPayload<LaserScan>(pickDisplayEnvelope(envelopes, displays, 'laser'));
  }, [envelopes, displays]);

  const laserOverlays = useMemo(
    () => resolveLaserOverlays(envelopes, displays),
    [envelopes, displays],
  );

  const cloudOverlay = useMemo(
    () => resolveCloudOverlay(envelopes, displays),
    [envelopes, displays],
  );

  const rangeOverlays = useMemo(
    () => resolveRangeOverlays(envelopes, displays),
    [envelopes, displays],
  );

  const pathStyle = useMemo(() => resolvePathStyle(displays), [displays]);

  const nav = useMemo(() => {
    return asPayload<NavPayload>(pickDisplayEnvelope(envelopes, displays, 'navigation'));
  }, [envelopes, displays]);

  const footprint = useMemo(() => {
    return asPayload<RobotFootprintJson>(
      pickDisplayEnvelope(envelopes, displays, 'footprint'),
    );
  }, [envelopes, displays]);

  const tfBuffer = useTfBufferStore((s) => s.byChild);
  const tf = useMemo(() => {
    const buffered = Object.values(tfBuffer);
    if (buffered.length) return { transforms: buffered };
    const envs = pickDisplayEnvelopes(envelopes, displays, 'tf');
    const extras = Object.values(envelopes).filter(
      (e) =>
        e.schema === SCHEMAS.TfTree &&
        !envs.some((x) => x.channel === e.channel),
    );
    const ordered = [...extras, ...envs].sort((a, b) => {
      const rank = (ch: string) =>
        /tf_static/i.test(ch) ? 0 : /\/tf$/i.test(ch) ? 2 : 1;
      return rank(a.channel) - rank(b.channel);
    });
    const transforms = mergeTfTransforms(
      ordered.map((e) => asPayload<TfTree>(e)?.transforms),
    );
    return transforms.length ? { transforms } : null;
  }, [tfBuffer, envelopes, displays]);

  const obstacles = useMemo(() => {
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
    }>(pickDisplayEnvelope(envelopes, displays, 'obstacles'));
  }, [envelopes, displays]);

  const vectorMap = useMemo(() => {
    return asPayload<{
      lanes?: { id: string; points: number[][] }[];
      keepouts?: { id: string; polygon: number[][] }[];
    }>(pickDisplayEnvelope(envelopes, displays, 'vectormap'));
  }, [envelopes, displays]);

  const prediction = useMemo(() => {
    return asPayload<{
      obstacles?: { id: number; trajectory?: { x: number; y: number }[] }[];
    }>(pickDisplayEnvelope(envelopes, displays, 'prediction'));
  }, [envelopes, displays]);

  const paintLayers = useMemo(
    () => effectiveMapLayers(layers, displays),
    [layers, displays],
  );

  useEffect(() => {
    let cancelled = false;
    if (!staticBasemap?.imageSrc) {
      setBasemapHandle(null);
      return;
    }
    void sharedStaticSlamCanvasCache.get(staticBasemap).then((h) => {
      if (!cancelled) setBasemapHandle(h);
    }).catch((err) => {
      console.warn('[orbisview] basemap load failed', err);
      if (!cancelled) setBasemapHandle(null);
    });
    return () => {
      cancelled = true;
    };
  }, [staticBasemap]);

  const goal =
    localGoal ??
    (nav?.has_goal !== false && nav?.goal ? nav.goal : null);

  useEffect(() => {
    // Do not fight an in-progress free pan.
    if (followRobot && pose && !viewPanRef.current) {
      setViewOffset({ x: pose.x, y: pose.y });
    }
  }, [followRobot, pose]);

  useEffect(() => {
    const host = hostRef.current;
    const canvas = canvasRef.current;
    if (!host || !canvas) return;
    const sync = () => {
      const w = Math.max(1, Math.floor(host.clientWidth));
      const h = Math.max(1, Math.floor(host.clientHeight));
      if (canvas.width !== w || canvas.height !== h) {
        canvas.width = w;
        canvas.height = h;
        setCanvasSize({ w, h });
      }
    };
    sync();
    const ro = new ResizeObserver(sync);
    ro.observe(host);
    return () => ro.disconnect();
  }, []);

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
      scale,
      viewOffset,
      layers: {
        grid: paintLayers.grid,
        basemap: paintLayers.basemap,
        map: paintLayers.map,
        costmap: paintLayers.costmap,
        semantic: paintLayers.semantic,
        vectormap: paintLayers.vectormap,
        poi: paintLayers.poi,
        draw: paintLayers.draw,
        path: paintLayers.path,
        robot: paintLayers.robot,
        footprint: paintLayers.footprint,
        obstacles: paintLayers.obstacles,
        prediction: paintLayers.prediction,
        laser: paintLayers.laser,
        tf: paintLayers.tf,
        pointcloud: paintLayers.pointcloud,
      },
      basemap: basemapHandle,
      pose,
      path,
      pathStyle,
      map,
      costmap,
      laser,
      lasers: laserOverlays.map((o) => ({
        scan: o.scan,
        color: o.style.color,
        size: o.style.size,
        alpha: o.style.alpha,
      })),
      cloud: cloudOverlay
        ? {
            points: cloudOverlay.points,
            colorMode: cloudOverlay.style.colorMode,
            size: cloudOverlay.style.size,
            alpha: cloudOverlay.style.alpha,
          }
        : null,
      ranges: rangeOverlays.map((o) => ({
        range: o.range.range,
        field_of_view: o.range.field_of_view ?? 0.25,
        color: hexToRgba(o.style.color, o.style.alpha),
        alpha: o.style.alpha,
      })),
      tf,
      obstacles,
      vectorMap,
      semanticZones,
      annotations: {
        pois: annPois,
        shapes: annShapes,
        draft: annDraft,
        draftPreview: annDraftPreview,
        selectedId: annSelectedId,
      },
      prediction,
      goal,
      footprint,
      defaultFootprint: DEFAULT_FOOTPRINT,
      measurePts: mapTool === 'measure' ? sketchPts : [],
      measurePreview: mapTool === 'measure' ? measurePreview : null,
      poseHandles:
        mapTool === 'nav' || mapTool === 'pick'
          ? sketchPts.map((p) => ({
              x: p.x,
              y: p.y,
              yaw: p.yaw ?? 0,
              label: p.label,
              ringM: p.ringM ?? 0.5,
              editing: !!p.editing,
            }))
          : [],
      routePts: waypoints.map((w, i) => ({
        x: w.x,
        y: w.y,
        yaw: w.yaw ?? 0,
        label: w.label ?? `#${i + 1}`,
        color: waypointColor(i),
        selected: w.id === selectedId,
        editing: wpEditRef.current?.id === w.id,
      })),
    });
  }, [
    pose,
    path,
    pathStyle,
    map,
    costmap,
    laser,
    laserOverlays,
    cloudOverlay,
    rangeOverlays,
    paintLayers,
    goal,
    footprint,
    tf,
    viewOffset,
    obstacles,
    vectorMap,
    semanticZones,
    annPois,
    annShapes,
    annDraft,
    annDraftPreview,
    annSelectedId,
    prediction,
    sketchPts,
    waypoints,
    selectedId,
    mapTool,
    measurePreview,
    canvasSize,
    scale,
    basemapHandle,
  ]);

  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      if (e.key !== 'Escape') return;
      if (mapTool === 'measure' && sketchPts.length) {
        setStatusMsg(`${useMapViewStore.getState().statusMsg} · 结束`);
      }
    };
    window.addEventListener('keydown', onKey);
    return () => window.removeEventListener('keydown', onKey);
  }, [mapTool, sketchPts.length, setStatusMsg]);

  const worldFromEvent = (e: React.MouseEvent<HTMLCanvasElement>) => {
    const canvas = canvasRef.current;
    if (!canvas) return null;
    const rect = canvas.getBoundingClientRect();
    const sx = ((e.clientX - rect.left) / rect.width) * canvas.width;
    const sy = ((e.clientY - rect.top) / rect.height) * canvas.height;
    const x = (sx - canvas.width / 2) / scale + viewOffset.x;
    const y = (canvas.height / 2 - sy) / scale + viewOffset.y;
    return { x, y };
  };

  const hitWaypoint = (w: { x: number; y: number }, radiusM = 0.65) => {
    const list = useWaypointStore.getState().waypoints;
    let best: { id: string; dist: number } | null = null;
    for (const wp of list) {
      const d = Math.hypot(wp.x - w.x, wp.y - w.y);
      if (d <= radiusM && (!best || d < best.dist)) best = { id: wp.id, dist: d };
    }
    return best?.id ?? null;
  };

  const pathLength = (pts: { x: number; y: number }[]) => {
    let dist = 0;
    for (let i = 1; i < pts.length; i++) {
      dist += Math.hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
    }
    return dist;
  };

  const yawDeg = (yaw: number) => ((yaw * 180) / Math.PI).toFixed(0);

  const updateDragSketch = (drag: PoseDrag) => {
    setSketchPts([
      {
        x: drag.x,
        y: drag.y,
        yaw: drag.yaw,
        label: drag.label,
        ringM: drag.ringM,
        editing: true,
      },
    ]);
    if (drag.tool === 'pick') {
      setStatusMsg(
        `取点 (${drag.x.toFixed(2)}, ${drag.y.toFixed(2)}) · 拖动设朝向 ${yawDeg(drag.yaw)}°` +
          (drag.moved ? '' : ' · 或滚轮转动'),
      );
      return;
    }
    setStatusMsg(
      `${drag.label} · 拖动箭头设朝向 ${yawDeg(drag.yaw)}°` +
        (drag.moved ? '' : ' · 或滚轮转动'),
    );
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
    addWaypoint(drag.x, drag.y, yaw, `#${n}`);
    setSketchPts([]);
    const list = useWaypointStore.getState().waypoints;
    if (list.length === 1) {
      setLocalGoal({ x: drag.x, y: drag.y, yaw });
      if (connected) {
        wsClient.send({ op: 'set_goal', x: drag.x, y: drag.y, yaw });
      }
      setStatusMsg(
        `目标点 (${drag.x.toFixed(2)}, ${drag.y.toFixed(2)}) yaw ${yawDeg(yaw)}°` +
          (!connected ? ' · offline' : ' · 已发送'),
      );
      return;
    }
    setStatusMsg(`导航点 #${list.length} · 共 ${list.length} 点 · 点发送下发路线`);
  };

  const onPointerDown = (e: React.PointerEvent<HTMLCanvasElement>) => {
    // Measure / draw: RMB finishes — suppress menu, do not pan.
    if ((mapTool === 'measure' || mapTool === 'draw') && e.button === 2) {
      e.preventDefault();
      return;
    }
    // Free pan in any direction: pan tool, middle, right, Shift/Space+left.
    const wantPan =
      mapTool === 'pan' ||
      e.button === 1 ||
      (e.button === 2 && mapTool !== 'poi') ||
      (e.button === 0 &&
        (e.shiftKey || spaceHeldRef.current) &&
        mapTool !== 'poi' &&
        mapTool !== 'draw');
    if (wantPan) {
      e.preventDefault();
      e.currentTarget.setPointerCapture(e.pointerId);
      useLayerStore.getState().setFollowRobot(false);
      viewPanRef.current = { lastX: e.clientX, lastY: e.clientY };
      setStatusMsg('拖动平移视图中…');
      return;
    }
    if (e.button !== 0) return;
    const w = worldFromEvent(e);
    if (!w) return;

    // Hit existing waypoint → select + edit (any tool except measure/pick/poi/draw)
    if (
      mapTool !== 'measure' &&
      mapTool !== 'pick' &&
      mapTool !== 'poi' &&
      mapTool !== 'draw'
    ) {
      const hitId = hitWaypoint(w);
      if (hitId) {
        const wp = useWaypointStore.getState().waypoints.find((x) => x.id === hitId);
        if (wp) {
          e.currentTarget.setPointerCapture(e.pointerId);
          selectWaypoint(hitId);
          wpEditRef.current = {
            id: hitId,
            originX: wp.x,
            originY: wp.y,
            yaw: wp.yaw ?? 0,
            mode: 'move',
            moved: false,
          };
          setStatusMsg(
            `选中 ${wp.label ?? hitId} · 拖动改位置 · 双击打开 Waypoints`,
          );
          return;
        }
      }
    }

    if (mapTool === 'poi') {
      e.currentTarget.setPointerCapture(e.pointerId);
      const hit = hitTestPoi(useAnnotationStore.getState().pois, w.x, w.y, 0.35);
      if (hit) {
        poiDragRef.current = { id: hit.id };
        useAnnotationStore.getState().setSelected(hit.id);
        setStatusMsg(`拖移 POI ${hit.label ?? hit.kind}`);
        return;
      }
      useAnnotationStore.getState().addPoi({
        x: w.x,
        y: w.y,
        kind: useAnnotationStore.getState().poiDefaultKind,
      });
      setStatusMsg(`已添加 POI (${w.x.toFixed(2)}, ${w.y.toFixed(2)})`);
      return;
    }

    if (mapTool === 'draw') {
      if (e.detail > 1) return; // let dblclick commit
      useAnnotationStore.getState().appendDraftPoint(w.x, w.y);
      const n = useAnnotationStore.getState().draft?.points.length ?? 0;
      setStatusMsg(`绘制点 ${n} · 双击/右击结束 · Esc 取消`);
      return;
    }

    if (mapTool !== 'nav' && mapTool !== 'pick') return;
    e.currentTarget.setPointerCapture(e.pointerId);

    const label = mapTool === 'pick' ? 'Pick' : `#${waypoints.length + 1}`;
    const drag: PoseDrag = {
      tool: mapTool,
      x: w.x,
      y: w.y,
      yaw: 0,
      ringM: 0.5,
      moved: false,
      label,
    };
    poseDragRef.current = drag;
    updateDragSketch(drag);
  };
  const onPointerMove = (e: React.PointerEvent<HTMLCanvasElement>) => {
    const pan = viewPanRef.current;
    if (pan) {
      const canvas = canvasRef.current;
      if (!canvas) return;
      const rect = canvas.getBoundingClientRect();
      const sx = canvas.width / Math.max(rect.width, 1);
      const sy = canvas.height / Math.max(rect.height, 1);
      const dx = (e.clientX - pan.lastX) * sx;
      const dy = (e.clientY - pan.lastY) * sy;
      pan.lastX = e.clientX;
      pan.lastY = e.clientY;
      setViewOffset((o) => ({
        x: o.x - dx / scale,
        y: o.y + dy / scale,
      }));
      return;
    }

    const w = worldFromEvent(e);
    if (!w) return;

    const edit = wpEditRef.current;
    if (edit) {
      const dist = Math.hypot(w.x - edit.originX, w.y - edit.originY);
      if (edit.mode === 'yaw' || dist > 0.5) {
        edit.mode = 'yaw';
        edit.moved = true;
        edit.yaw = Math.atan2(w.y - edit.originY, w.x - edit.originX);
        updateWaypoint(edit.id, { x: edit.originX, y: edit.originY, yaw: edit.yaw });
        setStatusMsg(`朝向 ${yawDeg(edit.yaw)}° · 松手确认`);
      } else if (dist > 0.03) {
        edit.mode = 'move';
        edit.moved = true;
        edit.originX = w.x;
        edit.originY = w.y;
        updateWaypoint(edit.id, { x: w.x, y: w.y });
        setStatusMsg(`移动 (${w.x.toFixed(2)}, ${w.y.toFixed(2)})`);
      }
      return;
    }

    const poiDrag = poiDragRef.current;
    if (poiDrag) {
      useAnnotationStore.getState().updatePoi(poiDrag.id, { x: w.x, y: w.y });
      setStatusMsg(`POI (${w.x.toFixed(2)}, ${w.y.toFixed(2)})`);
      return;
    }

    if (mapTool === 'draw' && useAnnotationStore.getState().draft) {
      useAnnotationStore.getState().setDraftPreview(w);
      return;
    }

    if (mapTool === 'measure' && sketchPts.length === 1 && !poseDragRef.current) {
      setMeasurePreview(w);
      return;
    }

    const drag = poseDragRef.current;
    if (!drag) return;
    const dist = Math.hypot(w.x - drag.x, w.y - drag.y);
    if (dist > 0.05) {
      drag.moved = true;
      drag.yaw = Math.atan2(w.y - drag.y, w.x - drag.x);
      drag.ringM = Math.max(0.4, Math.min(2.5, dist));
    }
    updateDragSketch({ ...drag });
  };

  const finishMeasureAt = (end: { x: number; y: number } | null) => {
    const cur = sketchRef.current;
    if (cur.length === 1 && end) {
      const pts = [cur[0], end];
      sketchRef.current = pts;
      setSketchPts(pts);
      setMeasurePreview(null);
      const dist = pathLength(pts);
      setStatusMsg(
        `测距 ${dist.toFixed(2)} m · (${pts[0].x.toFixed(2)}, ${pts[0].y.toFixed(2)}) → (${end.x.toFixed(2)}, ${end.y.toFixed(2)}) · 结果保留`,
      );
      return;
    }
    if (cur.length >= 2) {
      setStatusMsg(
        `测距 ${pathLength(cur).toFixed(2)} m · 结果保留 · 左键可重新开始`,
      );
      return;
    }
    setStatusMsg('测距：请先左键确定起点');
  };

  const onPointerUp = (e: React.PointerEvent<HTMLCanvasElement>) => {
    if (mapTool === 'measure' && e.button === 2) {
      const w = worldFromEvent(e);
      finishMeasureAt(measurePreviewRef.current ?? w);
      return;
    }

    if (viewPanRef.current) {
      viewPanRef.current = null;
      try {
        e.currentTarget.releasePointerCapture(e.pointerId);
      } catch {
        /* ignore */
      }
      setStatusMsg(
        mapTool === 'pan'
          ? '拖动视图：左键平移，滚轮缩放'
          : useMapViewStore.getState().statusMsg,
      );
      return;
    }

    if (poiDragRef.current) {
      poiDragRef.current = null;
      try {
        e.currentTarget.releasePointerCapture(e.pointerId);
      } catch {
        /* ignore */
      }
      setStatusMsg(useMapViewStore.getState().statusMsg);
      return;
    }

    if (wpEditRef.current) {
      const edit = wpEditRef.current;
      wpEditRef.current = null;
      try {
        e.currentTarget.releasePointerCapture(e.pointerId);
      } catch {
        /* ignore */
      }
      const wp = useWaypointStore.getState().waypoints.find((x) => x.id === edit.id);
      if (wp) {
        const list = useWaypointStore.getState().waypoints;
        if (list.length === 1 && connected) {
          setLocalGoal({ x: wp.x, y: wp.y, yaw: wp.yaw ?? 0 });
          wsClient.send({ op: 'set_goal', x: wp.x, y: wp.y, yaw: wp.yaw ?? 0 });
          setStatusMsg(
            `已更新目标 (${wp.x.toFixed(2)}, ${wp.y.toFixed(2)}) yaw ${yawDeg(wp.yaw ?? 0)}°`,
          );
        } else {
          setStatusMsg(
            `已更新 ${wp.label ?? '#'} (${wp.x.toFixed(2)}, ${wp.y.toFixed(2)}) yaw ${yawDeg(wp.yaw ?? 0)}°`,
          );
        }
      }
      return;
    }

    const drag = poseDragRef.current;
    if (!drag) return;
    poseDragRef.current = null;
    try {
      e.currentTarget.releasePointerCapture(e.pointerId);
    } catch {
      /* ignore */
    }
    finishPoseDrag(drag);
  };

  const onWheel = (e: React.WheelEvent<HTMLCanvasElement>) => {
    const edit = wpEditRef.current;
    if (edit) {
      e.preventDefault();
      edit.moved = true;
      edit.mode = 'yaw';
      edit.yaw += e.deltaY * 0.004;
      updateWaypoint(edit.id, { yaw: edit.yaw });
      setStatusMsg(`朝向 ${yawDeg(edit.yaw)}°`);
      return;
    }
    const sel = useWaypointStore.getState().selectedId;
    if (sel && !poseDragRef.current && mapTool !== 'measure' && mapTool !== 'pan') {
      const wp = useWaypointStore.getState().waypoints.find((x) => x.id === sel);
      if (wp) {
        e.preventDefault();
        const yaw = (wp.yaw ?? 0) + e.deltaY * 0.004;
        updateWaypoint(sel, { yaw });
        setStatusMsg(`选中点朝向 ${yawDeg(yaw)}°`);
        return;
      }
    }
    const drag = poseDragRef.current;
    if (drag) {
      e.preventDefault();
      drag.moved = true;
      drag.yaw += e.deltaY * 0.004;
      updateDragSketch({ ...drag });
      return;
    }
    // Default: zoom toward cursor
    e.preventDefault();
    const factor = e.deltaY < 0 ? 1.12 : 1 / 1.12;
    setScale((s) => clampScale(s * factor));
    useLayerStore.getState().setFollowRobot(false);
  };

  const onClick = (e: React.MouseEvent<HTMLCanvasElement>) => {
    if (mapTool === 'nav' || mapTool === 'pan' || mapTool === 'pick') return;
    const w = worldFromEvent(e);
    if (!w) return;

    if (mapTool === 'measure') {
      // Left click: start (or restart) a measurement — only the first point.
      setSketchPts([w]);
      setMeasurePreview(null);
      setStatusMsg(
        `测距起点 (${w.x.toFixed(2)}, ${w.y.toFixed(2)}) · 移动预览 · 右击落终点`,
      );
    }
  };

  const onDoubleClick = (e: React.MouseEvent<HTMLCanvasElement>) => {
    if (mapTool === 'draw') {
      e.preventDefault();
      const ok = useAnnotationStore.getState().commitDraft();
      setStatusMsg(ok ? '已完成绘制' : '点数不足，继续加点或 Esc 取消');
      return;
    }
    const w = worldFromEvent(e);
    if (!w) return;
    const hitId = hitWaypoint(w, 0.8);
    if (!hitId) return;
    e.preventDefault();
    selectWaypoint(hitId);
    wpEditRef.current = null;
    poseDragRef.current = null;
    useLayoutStore.getState().ensurePanel('waypoints');
    setStatusMsg('已打开 Waypoints · 可在列表中编辑');
  };

  const onContextMenu = (e: React.MouseEvent<HTMLCanvasElement>) => {
    e.preventDefault();
    if (mapTool === 'draw') {
      const ok = useAnnotationStore.getState().commitDraft();
      setStatusMsg(ok ? '已完成绘制' : '点数不足，继续加点或 Esc 取消');
      return;
    }
    if (wpEditRef.current) {
      wpEditRef.current = null;
      setStatusMsg('已取消编辑');
      return;
    }
    if (poseDragRef.current) {
      poseDragRef.current = null;
      setSketchPts([]);
      setStatusMsg('已取消落点');
      return;
    }
    if (mapTool === 'measure') {
      const w = worldFromEvent(e);
      finishMeasureAt(measurePreviewRef.current ?? w);
      return;
    }
    if (mapTool === 'nav') {
      setSketchPts([]);
      setStatusMsg('导航：落点设朝向；1 点发目标，多点发路线');
    }
  };

  const clearGoal = () => {
    setLocalGoal(null);
    setSketchPts([]);
    wsClient.send({ op: 'clear_goal' });
    setStatusMsg('已清除目标');
  };

  const clampScale = (s: number) => Math.max(MIN_SCALE, Math.min(MAX_SCALE, s));

  const zoomIn = () => {
    setScale((s) => clampScale(s * 1.25));
    useLayerStore.getState().setFollowRobot(false);
    setStatusMsg('已放大');
  };

  const zoomOut = () => {
    setScale((s) => clampScale(s / 1.25));
    useLayerStore.getState().setFollowRobot(false);
    setStatusMsg('已缩小');
  };

  const fitView = () => {
    const canvas = canvasRef.current;
    const w = canvas?.width ?? canvasSize.w;
    const h = canvas?.height ?? canvasSize.h;
    const pts: { x: number; y: number }[] = [];

    const pushGrid = (g: NonNullable<typeof map>) => {
      const ox = g.origin?.x ?? 0;
      const oy = g.origin?.y ?? 0;
      const res = g.resolution ?? 0.05;
      pts.push({ x: ox, y: oy });
      pts.push({ x: ox + g.width * res, y: oy + g.height * res });
    };

    if (staticBasemap && paintLayers.basemap) {
      const c = staticSlamCorners(staticBasemap);
      pts.push({ x: c.x0, y: c.y0 });
      pts.push({ x: c.x1, y: c.y1 });
    } else if (map && paintLayers.map) {
      pushGrid(map);
    } else if (costmap && paintLayers.costmap) {
      pushGrid(costmap);
    } else {
      if (pose) pts.push(pose);
      if (goal) pts.push(goal);
      waypoints.forEach((wp) => pts.push(wp));
      if (map) pushGrid(map);
    }

    if (!pts.length) {
      setScale(DEFAULT_SCALE);
      setViewOffset({ x: 0, y: 0 });
      useLayerStore.getState().setFollowRobot(false);
      setStatusMsg('自适应 · 无地图数据，已复位');
      return;
    }
    let minX = pts[0].x;
    let maxX = pts[0].x;
    let minY = pts[0].y;
    let maxY = pts[0].y;
    for (const p of pts) {
      minX = Math.min(minX, p.x);
      maxX = Math.max(maxX, p.x);
      minY = Math.min(minY, p.y);
      maxY = Math.max(maxY, p.y);
    }
    const pad = 1.2;
    const spanX = Math.max(2, maxX - minX) * pad;
    const spanY = Math.max(2, maxY - minY) * pad;
    const nextScale = clampScale(Math.min(w / spanX, h / spanY));
    setScale(nextScale);
    setViewOffset({ x: (minX + maxX) / 2, y: (minY + maxY) / 2 });
    useLayerStore.getState().setFollowRobot(false);
    setStatusMsg(`自适应 · scale ${nextScale.toFixed(0)}`);
  };

  const goalLabel = goal
    ? `goal (${goal.x.toFixed(2)}, ${goal.y.toFixed(2)}, ${yawDeg(goal.yaw ?? 0)}°)`
    : undefined;

  return (
    <div className="map-viewport map-primary" ref={hostRef}>
      {anyStale ? <div className="stale-badge map-float-badge">map data stale</div> : null}
      <MapInstrumentCluster />
      <MapFloorBar />
      <MapMappingHud />
      <MapFloatToolbar
        measureActive={mapTool === 'measure' && sketchPts.length > 0}
        onClearMeasure={() => {
          setSketchPts([]);
          setMeasurePreview(null);
          setStatusMsg('测距：左键起点，移动预览，右击落终点');
        }}
        goalLabel={goalLabel}
        onClearGoal={clearGoal}
        onZoomIn={zoomIn}
        onZoomOut={zoomOut}
        onFit={fitView}
      />
      <canvas
        ref={canvasRef}
        className="map-canvas map-canvas-fill"
        onClick={onClick}
        onDoubleClick={onDoubleClick}
        onContextMenu={onContextMenu}
        onPointerDown={onPointerDown}
        onPointerMove={onPointerMove}
        onPointerUp={onPointerUp}
        onPointerCancel={onPointerUp}
        onPointerLeave={() => {
          // Keep confirmed result; only drop live preview rubber-band.
          if (mapTool === 'measure' && sketchRef.current.length < 2) {
            setMeasurePreview(null);
          }
        }}
        onWheel={onWheel}
        style={{
          cursor:
            mapTool === 'pan' || spaceHeldRef.current ? 'grab' : 'crosshair',
          touchAction: 'none',
        }}
      />
    </div>
  );
}
