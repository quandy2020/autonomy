import type { View3DContext } from './createScene';
import type { View3DSceneInput } from './types';
import { updateCloud } from './layers/cloud';
import { updateFootprint } from './layers/footprint';
import { updateLaser } from './layers/laser';
import { updateOccupancyPlane } from './layers/occupancy';
import { updatePathAndGoal } from './layers/path';
import { updateRobot } from './layers/robot';

export function syncView3DScene(ctx: View3DContext, input: View3DSceneInput): void {
  const { layers, opts } = input;
  ctx.grid.visible = layers.grid;
  updateRobot(ctx, input.pose, layers.robot);
  updatePathAndGoal(ctx, input.pose, input.path, input.goal, layers.path);
  updateCloud(ctx, input.cloud, layers.pointcloud, opts.cloudColor);
  updateFootprint(ctx, input.pose, input.footprint, layers.footprint);
  updateOccupancyPlane(ctx.mapPlane, input.map, {
    opacity: opts.mapOpacity,
    yLift: 0.01,
    visible: layers.map,
  });
  updateOccupancyPlane(ctx.costmapPlane, input.costmap, {
    opacity: opts.mapOpacity * 0.85,
    yLift: 0.02,
    visible: layers.costmap,
  });
  updateLaser(ctx, input.pose, input.laser, opts.laserHeight, layers.laser);
}
