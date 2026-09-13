import { useEffect, useRef } from 'react';
import * as THREE from 'three';
import { useDataStore } from '@/store/dataStore';
import { useLayerStore } from '@/store/layoutStore';
import { SCHEMAS } from '@/store/websocket/types';

type SceneCtx = {
  renderer: THREE.WebGLRenderer;
  scene: THREE.Scene;
  camera: THREE.PerspectiveCamera;
  robot: THREE.Mesh;
  goal: THREE.Mesh;
  goalLine: THREE.Line;
  path: THREE.Line;
  grid: THREE.GridHelper;
  cloud: THREE.Points;
  yaw: number;
  pitch: number;
  distance: number;
};

export function View3DPanel() {
  const mountRef = useRef<HTMLDivElement>(null);
  const envelopes = useDataStore((s) => s.envelopes);
  const layers = useLayerStore();
  const sceneRef = useRef<SceneCtx | null>(null);

  useEffect(() => {
    const mount = mountRef.current;
    if (!mount) return;
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0f1419);
    const camera = new THREE.PerspectiveCamera(55, 720 / 420, 0.1, 200);
    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(720, 420);
    mount.appendChild(renderer.domElement);

    const grid = new THREE.GridHelper(20, 20, 0x334455, 0x1f2a33);
    scene.add(grid);
    const light = new THREE.DirectionalLight(0xffffff, 1);
    light.position.set(5, 10, 7);
    scene.add(light);
    scene.add(new THREE.AmbientLight(0x6688aa, 0.5));

    const robot = new THREE.Mesh(
      new THREE.ConeGeometry(0.2, 0.5, 12),
      new THREE.MeshStandardMaterial({ color: 0x69f0ae }),
    );
    robot.rotation.x = Math.PI / 2;
    scene.add(robot);

    const goal = new THREE.Mesh(
      new THREE.ConeGeometry(0.25, 0.55, 3),
      new THREE.MeshStandardMaterial({ color: 0xff7043 }),
    );
    goal.visible = false;
    scene.add(goal);

    const goalLine = new THREE.Line(
      new THREE.BufferGeometry(),
      new THREE.LineDashedMaterial({ color: 0xff7043, dashSize: 0.2, gapSize: 0.1 }),
    );
    goalLine.visible = false;
    scene.add(goalLine);

    const pathGeom = new THREE.BufferGeometry();
    const path = new THREE.Line(
      pathGeom,
      new THREE.LineBasicMaterial({ color: 0x4fc3f7 }),
    );
    scene.add(path);

    const cloudGeom = new THREE.BufferGeometry();
    cloudGeom.setAttribute('position', new THREE.Float32BufferAttribute([], 3));
    cloudGeom.setAttribute('color', new THREE.Float32BufferAttribute([], 3));
    const cloud = new THREE.Points(
      cloudGeom,
      new THREE.PointsMaterial({ size: 0.06, vertexColors: true }),
    );
    scene.add(cloud);

    const ctx: SceneCtx = {
      renderer,
      scene,
      camera,
      robot,
      goal,
      goalLine,
      path,
      grid,
      cloud,
      yaw: 0.8,
      pitch: 0.6,
      distance: 8,
    };
    sceneRef.current = ctx;

    const updateCamera = () => {
      const cp = Math.cos(ctx.pitch);
      camera.position.set(
        ctx.distance * cp * Math.sin(ctx.yaw),
        ctx.distance * Math.sin(ctx.pitch),
        ctx.distance * cp * Math.cos(ctx.yaw),
      );
      camera.lookAt(0, 0, 0);
    };
    updateCamera();

    let dragging = false;
    let lastX = 0;
    let lastY = 0;
    const onDown = (e: MouseEvent) => {
      dragging = true;
      lastX = e.clientX;
      lastY = e.clientY;
    };
    const onUp = () => {
      dragging = false;
    };
    const onMove = (e: MouseEvent) => {
      if (!dragging) return;
      ctx.yaw -= (e.clientX - lastX) * 0.01;
      ctx.pitch = Math.max(0.1, Math.min(1.4, ctx.pitch + (e.clientY - lastY) * 0.01));
      lastX = e.clientX;
      lastY = e.clientY;
      updateCamera();
    };
    const onWheel = (e: WheelEvent) => {
      e.preventDefault();
      ctx.distance = Math.max(2, Math.min(40, ctx.distance + e.deltaY * 0.01));
      updateCamera();
    };
    renderer.domElement.addEventListener('mousedown', onDown);
    window.addEventListener('mouseup', onUp);
    window.addEventListener('mousemove', onMove);
    renderer.domElement.addEventListener('wheel', onWheel, { passive: false });

    let raf = 0;
    const tick = () => {
      renderer.render(scene, camera);
      raf = requestAnimationFrame(tick);
    };
    tick();

    return () => {
      cancelAnimationFrame(raf);
      renderer.domElement.removeEventListener('mousedown', onDown);
      window.removeEventListener('mouseup', onUp);
      window.removeEventListener('mousemove', onMove);
      renderer.domElement.removeEventListener('wheel', onWheel);
      renderer.dispose();
      mount.removeChild(renderer.domElement);
      sceneRef.current = null;
    };
  }, []);

  useEffect(() => {
    const ctx = sceneRef.current;
    if (!ctx) return;
    ctx.grid.visible = layers.grid;
    ctx.robot.visible = layers.robot;
    ctx.path.visible = layers.path;
    ctx.cloud.visible = layers.pointcloud;

    const poseEnv = Object.values(envelopes).find((e) => e.schema === SCHEMAS.Pose2D);
    const pose = poseEnv?.payload as { x: number; y: number; yaw?: number } | undefined;
    if (pose && layers.robot) {
      ctx.robot.position.set(pose.x, 0.2, -pose.y);
      ctx.robot.rotation.z = -(pose.yaw ?? 0);
    }

    const pathEnv = Object.values(envelopes).find((e) => e.schema === SCHEMAS.Path2D);
    const path = pathEnv?.payload as { poses: { x: number; y: number }[] } | undefined;
    if (path?.poses && layers.path) {
      const pts = path.poses.map((p) => new THREE.Vector3(p.x, 0.05, -p.y));
      ctx.path.geometry.setFromPoints(pts);
    }

    const navEnv = Object.values(envelopes).find((e) => e.schema === SCHEMAS.Navigation);
    const nav = navEnv?.payload as
      | { has_goal?: boolean; goal?: { x: number; y: number } }
      | undefined;
    const showGoal = !!(nav?.goal && nav.has_goal !== false);
    ctx.goal.visible = showGoal;
    ctx.goalLine.visible = showGoal && !!pose;
    if (showGoal && nav?.goal) {
      ctx.goal.position.set(nav.goal.x, 0.3, -nav.goal.y);
      if (pose) {
        ctx.goalLine.geometry.setFromPoints([
          new THREE.Vector3(pose.x, 0.15, -pose.y),
          new THREE.Vector3(nav.goal.x, 0.15, -nav.goal.y),
        ]);
        ctx.goalLine.computeLineDistances();
      }
    }

    const cloudEnv = Object.values(envelopes).find((e) => e.schema === SCHEMAS.PointCloud2);
    const cloud = cloudEnv?.payload as
      | { points: { x: number; y: number; z: number; i?: number }[] }
      | undefined;
    if (cloud?.points && layers.pointcloud) {
      const n = cloud.points.length;
      const positions = new Float32Array(n * 3);
      const colors = new Float32Array(n * 3);
      for (let i = 0; i < n; i++) {
        const p = cloud.points[i];
        positions[i * 3] = p.x;
        positions[i * 3 + 1] = p.z;
        positions[i * 3 + 2] = -p.y;
        const inten = p.i ?? 0.7;
        colors[i * 3] = 0.2 + 0.8 * inten;
        colors[i * 3 + 1] = 0.6 * inten;
        colors[i * 3 + 2] = 1 - 0.5 * inten;
      }
      ctx.cloud.geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
      ctx.cloud.geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
      ctx.cloud.geometry.computeBoundingSphere();
    }
  }, [envelopes, layers]);

  const cloudStale = Object.values(envelopes).find((e) => e.schema === SCHEMAS.PointCloud2)?.stale;

  return (
    <div className="panel">
      {cloudStale ? <div className="stale-badge">pointcloud stale</div> : null}
      <div ref={mountRef} />
      <p className="hint">drag rotate · wheel zoom · orange = nav goal</p>
    </div>
  );
}
