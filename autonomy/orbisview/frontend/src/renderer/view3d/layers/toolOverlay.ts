import * as THREE from 'three';
import type { View3DContext } from '../createScene';
import { toThree } from '../coords';

export interface ToolOverlayPoint {
  x: number;
  y: number;
  yaw?: number;
  color?: number;
  label?: string;
}

export interface ToolOverlayInput {
  points: ToolOverlayPoint[];
  preview?: { x: number; y: number } | null;
  /** Connect points with a line (measure / route sketch). */
  connect?: boolean;
  dashedPreview?: boolean;
}

function disposeObject(o: THREE.Object3D): void {
  if ((o as THREE.Sprite).isSprite) {
    const mat = (o as THREE.Sprite).material as THREE.SpriteMaterial;
    mat.map?.dispose();
    mat.dispose();
    return;
  }
  const mesh = o as THREE.Mesh;
  if (mesh.geometry) mesh.geometry.dispose();
  const mat = mesh.material as THREE.Material | THREE.Material[] | undefined;
  if (!mat) return;
  if (Array.isArray(mat)) mat.forEach((m) => m.dispose());
  else mat.dispose();
}

function clearGroup(group: THREE.Group): void {
  while (group.children.length) {
    const child = group.children[0];
    group.remove(child);
    child.traverse(disposeObject);
  }
}

function makeDistanceSprite(text: string): THREE.Sprite {
  const canvas = document.createElement('canvas');
  canvas.width = 256;
  canvas.height = 64;
  const c = canvas.getContext('2d')!;
  c.clearRect(0, 0, canvas.width, canvas.height);
  c.font = 'bold 28px ui-sans-serif, system-ui, sans-serif';
  c.textAlign = 'center';
  c.textBaseline = 'middle';
  c.lineWidth = 6;
  c.strokeStyle = 'rgba(11,15,20,0.85)';
  c.fillStyle = '#fff59d';
  c.strokeText(text, 128, 32);
  c.fillText(text, 128, 32);
  const tex = new THREE.CanvasTexture(canvas);
  tex.needsUpdate = true;
  const mat = new THREE.SpriteMaterial({
    map: tex,
    transparent: true,
    depthTest: false,
  });
  const sprite = new THREE.Sprite(mat);
  sprite.scale.set(1.6, 0.4, 1);
  return sprite;
}

/** Draw temporary measure / nav-AB / place preview markers. */
export function updateToolOverlay(ctx: View3DContext, input: ToolOverlayInput): void {
  const g = ctx.toolOverlay;
  clearGroup(g);

  const { points, preview, connect, dashedPreview } = input;
  if (connect && points.length > 1) {
    const pts = points.map((p) => {
      const t = toThree(p.x, p.y, 0.08);
      return new THREE.Vector3(t.x, t.y, t.z);
    });
    const line = new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(pts),
      new THREE.LineBasicMaterial({ color: 0xffee58, transparent: true, opacity: 0.85 }),
    );
    g.add(line);

    // Persist mid-segment distance labels (measure result stays until cleared).
    for (let i = 1; i < points.length; i++) {
      const a = points[i - 1];
      const b = points[i];
      const seg = Math.hypot(b.x - a.x, b.y - a.y);
      const mid = toThree((a.x + b.x) / 2, (a.y + b.y) / 2, 0.35);
      const sprite = makeDistanceSprite(`${seg.toFixed(2)} m`);
      sprite.position.set(mid.x, mid.y, mid.z);
      g.add(sprite);
    }
  }

  points.forEach((p) => {
    const color = p.color ?? (connect ? 0xffee58 : 0x80cbc4);
    const marker = new THREE.Mesh(
      new THREE.SphereGeometry(0.12, 12, 12),
      new THREE.MeshStandardMaterial({ color, emissive: color, emissiveIntensity: 0.2 }),
    );
    const t = toThree(p.x, p.y, 0.12);
    marker.position.set(t.x, t.y, t.z);
    g.add(marker);

    if (p.yaw != null) {
      const dir = new THREE.ArrowHelper(
        new THREE.Vector3(Math.cos(p.yaw), 0, -Math.sin(p.yaw)),
        new THREE.Vector3(t.x, t.y, t.z),
        0.55,
        color,
        0.16,
        0.1,
      );
      g.add(dir);
    }
  });

  if (preview && points.length) {
    const last = points[points.length - 1];
    const a = toThree(last.x, last.y, 0.08);
    const b = toThree(preview.x, preview.y, 0.08);
    const geom = new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(a.x, a.y, a.z),
      new THREE.Vector3(b.x, b.y, b.z),
    ]);
    const mat = dashedPreview
      ? new THREE.LineDashedMaterial({
          color: 0xfff176,
          dashSize: 0.15,
          gapSize: 0.1,
          transparent: true,
          opacity: 0.9,
        })
      : new THREE.LineBasicMaterial({ color: 0xfff176, transparent: true, opacity: 0.9 });
    const line = new THREE.Line(geom, mat);
    if (dashedPreview) line.computeLineDistances();
    g.add(line);

    const seg = Math.hypot(preview.x - last.x, preview.y - last.y);
    const mid = toThree((last.x + preview.x) / 2, (last.y + preview.y) / 2, 0.35);
    const sprite = makeDistanceSprite(`${seg.toFixed(2)} m`);
    sprite.material.opacity = 0.85;
    sprite.position.set(mid.x, mid.y, mid.z);
    g.add(sprite);
  }

  g.visible = points.length > 0 || !!preview;
}
