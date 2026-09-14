import * as THREE from 'three';
import type { View3DContext } from '../createScene';
import { toThree } from '../coords';
import type { View3DWaypoint } from '../types';

function hexToInt(hex: string): number {
  const h = hex.replace('#', '');
  return parseInt(h.length === 3 ? h.split('').map((c) => c + c).join('') : h, 16);
}

function disposeObject(obj: THREE.Object3D): void {
  obj.traverse((o) => {
    const mesh = o as THREE.Mesh;
    if (mesh.geometry) mesh.geometry.dispose();
    const mat = mesh.material as THREE.Material | THREE.Material[] | undefined;
    if (!mat) return;
    if (Array.isArray(mat)) mat.forEach((m) => m.dispose());
    else mat.dispose();
  });
}

function makeMarker(color: string, selected: boolean): THREE.Group {
  const g = new THREE.Group();
  const body = new THREE.Mesh(
    new THREE.ConeGeometry(selected ? 0.22 : 0.18, selected ? 0.5 : 0.42, 10),
    new THREE.MeshStandardMaterial({
      color: hexToInt(color),
      emissive: selected ? 0xffee58 : 0x000000,
      emissiveIntensity: selected ? 0.35 : 0,
    }),
  );
  body.rotation.x = Math.PI;
  body.position.y = selected ? 0.25 : 0.21;
  body.name = 'wp-body';
  g.add(body);

  if (selected) {
    const ring = new THREE.Mesh(
      new THREE.RingGeometry(0.28, 0.36, 24),
      new THREE.MeshBasicMaterial({
        color: 0xffee58,
        side: THREE.DoubleSide,
        transparent: true,
        opacity: 0.9,
      }),
    );
    ring.rotation.x = -Math.PI / 2;
    ring.position.y = 0.02;
    ring.name = 'wp-halo';
    g.add(ring);
  }

  const arrow = new THREE.ArrowHelper(
    new THREE.Vector3(1, 0, 0),
    new THREE.Vector3(0, 0.08, 0),
    selected ? 0.7 : 0.55,
    hexToInt(color),
    0.18,
    0.1,
  );
  arrow.name = 'wp-arrow';
  g.add(arrow);
  return g;
}

function signature(waypoints: View3DWaypoint[]): string {
  return waypoints.map((w) => `${w.id}:${w.color}:${w.selected ? 1 : 0}`).join('|');
}

export function updateWaypoints(
  ctx: View3DContext,
  waypoints: View3DWaypoint[],
  visible: boolean,
): void {
  ctx.waypoints.visible = visible && waypoints.length > 0;
  ctx.waypointRoute.visible = visible && waypoints.length > 1;

  const sig = signature(waypoints);
  const prevSig = (ctx.waypoints.userData.sig as string | undefined) ?? '';
  if (sig !== prevSig) {
    while (ctx.waypoints.children.length) {
      const child = ctx.waypoints.children[0];
      ctx.waypoints.remove(child);
      disposeObject(child);
    }
    waypoints.forEach((wp, i) => {
      const marker = makeMarker(wp.color, wp.selected);
      marker.userData.wpId = wp.id;
      marker.userData.wpIndex = i;
      ctx.waypoints.add(marker);
    });
    ctx.waypoints.userData.sig = sig;
  }

  waypoints.forEach((wp, i) => {
    const marker = ctx.waypoints.children[i] as THREE.Group | undefined;
    if (!marker) return;
    const t = toThree(wp.x, wp.y, 0);
    marker.position.set(t.x, t.y, t.z);
    marker.rotation.y = -(wp.yaw ?? 0);
  });

  if (waypoints.length > 1) {
    const pts = waypoints.map((wp) => {
      const t = toThree(wp.x, wp.y, 0.06);
      return new THREE.Vector3(t.x, t.y, t.z);
    });
    ctx.waypointRoute.geometry.dispose();
    ctx.waypointRoute.geometry = new THREE.BufferGeometry().setFromPoints(pts);
  }
}
