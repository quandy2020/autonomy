import * as THREE from 'three';
import type { OccupancyGridJson } from '../../map2d/types';
import {
  OccupancyTextureCache,
  type OccupancyPaintMode,
} from '../../map2d/occupancyTexture';
import { toThree } from '../coords';

const mapCache = new OccupancyTextureCache();
const costmapCache = new OccupancyTextureCache();

export function updateOccupancyPlane(
  plane: THREE.Mesh,
  grid: OccupancyGridJson | null,
  opts: {
    opacity: number;
    yLift: number;
    visible: boolean;
    mode?: OccupancyPaintMode;
  },
): void {
  const mode = opts.mode ?? 'map';
  plane.visible = opts.visible && !!grid;
  if (!opts.visible || !grid) return;

  const cache = mode === 'costmap' ? costmapCache : mapCache;
  const tex = cache.get(grid, mode);
  if (!tex) {
    plane.visible = false;
    return;
  }

  const prev = plane.material as THREE.MeshBasicMaterial;
  if (prev.map) prev.map.dispose();
  const texture = new THREE.CanvasTexture(tex.canvas);
  // Canvas row 0 = high world Y (Autoviz); keep upright on the XY plane after -X rot.
  texture.flipY = false;
  texture.magFilter = THREE.NearestFilter;
  texture.minFilter = THREE.NearestFilter;
  texture.needsUpdate = true;
  prev.map = texture;
  prev.opacity = opts.opacity;
  prev.transparent = true;
  prev.needsUpdate = true;

  plane.geometry.dispose();
  plane.geometry = new THREE.PlaneGeometry(tex.worldW, tex.worldH);
  plane.rotation.x = -Math.PI / 2;
  const center = toThree(
    tex.originX + tex.worldW / 2,
    tex.originY + tex.worldH / 2,
    opts.yLift,
  );
  plane.position.set(center.x, center.y, center.z);
}
