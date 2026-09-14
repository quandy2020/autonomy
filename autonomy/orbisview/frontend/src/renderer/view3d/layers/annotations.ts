import * as THREE from 'three';
import {
  poiKindColor,
  type MapDrawShape,
  type MapPoi,
} from '../../map2d/annotations';
import type { AnnotationDraft } from '@/store/annotationStore';
import { toThree } from '../coords';

function keyOf(
  pois: MapPoi[],
  shapes: MapDrawShape[],
  draft: AnnotationDraft | null,
  selectedId: string | null,
): string {
  return [
    selectedId ?? '',
    draft ? `${draft.kind}:${draft.points.length}` : '',
    ...pois.map((p) => `${p.id}:${p.x.toFixed(3)}:${p.y.toFixed(3)}:${p.kind}`),
    ...shapes.map((s) => `${s.id}:${s.kind}:${s.points.length}`),
  ].join('|');
}

let lastKey = '';

export function updateAnnotations(
  group: THREE.Group,
  input: {
    pois: MapPoi[];
    shapes: MapDrawShape[];
    draft: AnnotationDraft | null;
    selectedId: string | null;
    showPoi: boolean;
    showDraw: boolean;
  },
): void {
  const { pois, shapes, draft, selectedId, showPoi, showDraw } = input;
  const visible = (showPoi && pois.length > 0) || (showDraw && (shapes.length > 0 || !!draft));
  group.visible = visible;
  if (!visible) {
    clearGroup(group);
    lastKey = '';
    return;
  }

  const key = keyOf(
    showPoi ? pois : [],
    showDraw ? shapes : [],
    showDraw ? draft : null,
    selectedId,
  );
  if (key === lastKey) return;
  lastKey = key;
  clearGroup(group);

  if (showDraw) {
    for (const s of shapes) {
      addShape(group, s.points, s.kind === 'polygon', '#ffa726', 0.25);
    }
    if (draft && draft.points.length >= 2) {
      addShape(group, draft.points, draft.kind === 'polygon', '#81d4fa', 0.2);
    }
  }

  if (showPoi) {
    for (const p of pois) {
      const color = new THREE.Color(p.color ?? poiKindColor(p.kind));
      const mesh = new THREE.Mesh(
        new THREE.ConeGeometry(p.id === selectedId ? 0.18 : 0.14, 0.35, 10),
        new THREE.MeshBasicMaterial({ color }),
      );
      const pos = toThree(p.x, p.y, 0.02);
      mesh.position.set(pos.x, pos.y + 0.12, pos.z);
      group.add(mesh);
    }
  }
}

function addShape(
  group: THREE.Group,
  points: [number, number][],
  close: boolean,
  hex: string,
  opacity: number,
) {
  if (close && points.length >= 3) {
    const shape = new THREE.Shape();
    points.forEach(([x, y], i) => {
      if (i === 0) shape.moveTo(x, y);
      else shape.lineTo(x, y);
    });
    const mesh = new THREE.Mesh(
      new THREE.ShapeGeometry(shape),
      new THREE.MeshBasicMaterial({
        color: hex,
        transparent: true,
        opacity,
        depthWrite: false,
        side: THREE.DoubleSide,
      }),
    );
    mesh.rotation.x = -Math.PI / 2;
    mesh.position.y = 0.018;
    group.add(mesh);
  }
  const pts = points.map(([x, y]) => {
    const p = toThree(x, y, 0.019);
    return new THREE.Vector3(p.x, p.y, p.z);
  });
  if (close && pts.length) pts.push(pts[0].clone());
  if (pts.length < 2) return;
  group.add(
    new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(pts),
      new THREE.LineBasicMaterial({ color: hex }),
    ),
  );
}

function clearGroup(group: THREE.Group) {
  while (group.children.length) {
    const c = group.children[0];
    group.remove(c);
    c.traverse((o) => {
      const mesh = o as THREE.Mesh | THREE.Line;
      if (mesh.geometry) mesh.geometry.dispose();
      const mat = (mesh as THREE.Mesh).material;
      if (!mat) return;
      if (Array.isArray(mat)) mat.forEach((m) => m.dispose());
      else (mat as THREE.Material).dispose();
    });
  }
}
