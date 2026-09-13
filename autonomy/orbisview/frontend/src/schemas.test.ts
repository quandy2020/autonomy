import { describe, expect, it } from 'vitest';
import { SCHEMAS, STALE_THRESHOLD_MS } from './store/websocket/types';
import { registerBuiltinPanels } from './components';
import { listPanels } from './components/registry';

describe('render schemas', () => {
  it('matches backend schema ids', () => {
    expect(SCHEMAS.Pose2D).toBe('orbisview.render.Pose2D');
    expect(SCHEMAS.OccupancyGrid).toBe('orbisview.render.OccupancyGrid');
    expect(SCHEMAS.RobotFootprint).toBe('orbisview.render.RobotFootprint');
    expect(SCHEMAS.Image).toBe('orbisview.render.Image');
    expect(SCHEMAS.PointCloud2).toBe('orbisview.render.PointCloud2');
    expect(SCHEMAS.DepthImage).toBe('orbisview.render.DepthImage');
    expect(SCHEMAS.Exploration).toBe('orbisview.render.Exploration');
    expect(SCHEMAS.Navigation).toBe('orbisview.render.Navigation');
    expect(SCHEMAS.Mapping).toBe('orbisview.render.Mapping');
    expect(SCHEMAS.Twist2D).toBe('orbisview.render.Twist2D');
    expect(SCHEMAS.ChassisState).toBe('orbisview.render.ChassisState');
    expect(SCHEMAS.ObstacleArray).toBe('orbisview.render.ObstacleArray');
    expect(SCHEMAS.WorldState).toBe('orbisview.render.WorldState');
    expect(SCHEMAS.RoutePath).toBe('orbisview.render.RoutePath');
    expect(SCHEMAS.VectorMap).toBe('orbisview.render.VectorMap');
    expect(SCHEMAS.PredictionObstacles).toBe('orbisview.render.PredictionObstacles');
    expect(SCHEMAS.PlanningDebug).toBe('orbisview.render.PlanningDebug');
  });

  it('defines stale threshold', () => {
    expect(STALE_THRESHOLD_MS).toBe(2000);
  });
});

describe('panel registry', () => {
  it('registers builtin panels including DV+ parity set', () => {
    registerBuiltinPanels();
    const ids = listPanels().map((p) => p.id);
    expect(ids).toContain('map2d');
    expect(ids).toContain('dashboard');
    expect(ids).toContain('charts');
    expect(ids).toContain('components');
    expect(ids).toContain('hmi');
    expect(ids).toContain('routing');
    expect(ids).toContain('mode_settings');
    expect(ids).toContain('module_delay');
    expect(ids).toContain('resources');
    expect(ids).toContain('pnc');
  });
});

describe('layout persist keys', () => {
  it('uses versioned localStorage names', () => {
    expect('orbisview-layout-v8').toMatch(/^orbisview-layout-v\d+$/);
    expect('orbisview-layers-v5').toMatch(/^orbisview-layers-v\d+$/);
  });

  it('uses versioned view3d opts key', () => {
    expect('orbisview-view3d-opts-v1').toMatch(/^orbisview-view3d-opts-v\d+$/);
  });
});
