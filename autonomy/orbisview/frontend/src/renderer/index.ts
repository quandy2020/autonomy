/**
 * Map2D / View3D / shared renderer helpers.
 */
export { makeWorldToScreen } from './map2d/coords';
export { paintMap2DScene } from './map2d/drawScene';
export type { Map2DSceneInput } from './map2d/drawScene';
export {
  drawFootprint,
  resolveFootprintPoints,
} from './map2d/drawFootprint';
export { drawOccupancyGrid } from './map2d/drawOccupancy';
export { drawMapHud } from './map2d/drawHud';
export {
  toThree,
  createView3DScene,
  syncView3DScene,
  createCameraController,
} from './view3d';
export type {
  View3DContext,
  View3DSceneInput,
  View3DOpts,
  CameraController,
} from './view3d';
