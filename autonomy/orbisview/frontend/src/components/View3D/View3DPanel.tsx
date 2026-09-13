import { useEffect, useRef } from 'react';
import * as THREE from 'three';
import { createView3DScene, type View3DContext } from '@/renderer/view3d';
import { useDataStore } from '@/store/dataStore';
import { useLayerStore } from '@/store/layoutStore';
import { SCHEMAS } from '@/store/websocket/types';

export function View3DPanel() {
  const mountRef = useRef<HTMLDivElement>(null);
  const envelopes = useDataStore((s) => s.envelopes);
  const layers = useLayerStore();
  const sceneRef = useRef<View3DContext | null>(null);
  const camRef = useRef({ yaw: 0.8, pitch: 0.6, distance: 8 });

  useEffect(() => {
    const mount = mountRef.current;
    if (!mount) return;
    const ctx = createView3DScene(mount);
    sceneRef.current = ctx;

    const updateCamera = () => {
      const { yaw, pitch, distance } = camRef.current;
      const cp = Math.cos(pitch);
      ctx.camera.position.set(
        distance * cp * Math.sin(yaw),
        distance * Math.sin(pitch),
        distance * cp * Math.cos(yaw),
      );
      ctx.camera.lookAt(0, 0, 0);
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
      camRef.current.yaw -= (e.clientX - lastX) * 0.01;
      camRef.current.pitch = Math.max(
        0.1,
        Math.min(1.4, camRef.current.pitch + (e.clientY - lastY) * 0.01),
      );
      lastX = e.clientX;
      lastY = e.clientY;
      updateCamera();
    };
    const onWheel = (e: WheelEvent) => {
      e.preventDefault();
      camRef.current.distance = Math.max(
        2,
        Math.min(40, camRef.current.distance + e.deltaY * 0.01),
      );
      updateCamera();
    };
    ctx.renderer.domElement.addEventListener('mousedown', onDown);
    window.addEventListener('mouseup', onUp);
    window.addEventListener('mousemove', onMove);
    ctx.renderer.domElement.addEventListener('wheel', onWheel, { passive: false });

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
      ctx.renderer.domElement.removeEventListener('mousedown', onDown);
      window.removeEventListener('mouseup', onUp);
      window.removeEventListener('mousemove', onMove);
      ctx.renderer.domElement.removeEventListener('wheel', onWheel);
      ctx.dispose();
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
    <div className="panel view3d-panel">
      {cloudStale ? <div className="stale-badge">pointcloud stale</div> : null}
      <div className="view3d-host" ref={mountRef} />
      <p className="hint">drag rotate · wheel zoom · orange = nav goal</p>
    </div>
  );
}
