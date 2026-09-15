import * as THREE from 'three';
import type { StaticSlamCanvasHandle } from '../../map2d/staticSlam';
import { toThree } from '../coords';

export function updateBasemapPlane(
  plane: THREE.Mesh,
  handle: StaticSlamCanvasHandle | null,
  visible: boolean,
): void {
  plane.visible = visible && !!handle;
  if (!visible || !handle) return;

  const prev = plane.material as THREE.MeshBasicMaterial;
  if (prev.map) prev.map.dispose();
  const texture = new THREE.CanvasTexture(handle.canvas);
  // Same as occupancy: image/canvas top = high map Y → flipY so it lands on v=1.
  texture.flipY = true;
  texture.magFilter = THREE.NearestFilter;
  texture.minFilter = THREE.NearestFilter;
  texture.needsUpdate = true;
  prev.map = texture;
  prev.transparent = true;
  prev.opacity = 1;
  prev.needsUpdate = true;

  plane.geometry.dispose();
  plane.geometry = new THREE.PlaneGeometry(handle.worldW, handle.worldH);
  plane.rotation.x = -Math.PI / 2;
  const center = toThree(
    handle.originX + handle.worldW / 2,
    handle.originY + handle.worldH / 2,
    0.005,
  );
  plane.position.set(center.x, center.y, center.z);
}
