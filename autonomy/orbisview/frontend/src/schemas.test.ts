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
    expect(SCHEMAS.SemanticZoneArray).toBe('orbisview.render.SemanticZoneArray');
    expect(SCHEMAS.FloorInfoArray).toBe('orbisview.render.FloorInfoArray');
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
    expect(ids).toContain('map');
    expect(ids).not.toContain('map2d');
    expect(ids).not.toContain('view3d');
    expect(ids).toContain('dashboard');
    expect(ids).toContain('charts');
    expect(ids).toContain('components');
    expect(ids).toContain('hmi');
    expect(ids).toContain('routing');
    expect(ids).toContain('mode_settings');
    expect(ids).toContain('module_delay');
    expect(ids).toContain('resources');
    expect(ids).toContain('pnc');
    expect(ids).toContain('ops');
    expect(ids).toContain('teleop');
  });

  it('assigns categories for catalog grouping', () => {
    registerBuiltinPanels();
    const panels = listPanels();
    expect(panels.every((p) => !!p.category)).toBe(true);
    expect(panels.find((p) => p.id === 'map')?.category).toBe('viz');
    expect(panels.find((p) => p.id === 'view3d')).toBeUndefined();
    expect(panels.find((p) => p.id === 'image')?.category).toBe('sensor');
    expect(panels.find((p) => p.id === 'teleop')?.category).toBe('robot');
    expect(panels.find((p) => p.id === 'image')?.allowMultiple).toBe(true);
    expect(panels.find((p) => p.id === 'navigation')?.category).toBe('planning');
    expect(panels.find((p) => p.id === 'diagnostics')?.category).toBe('monitor');
    expect(panels.find((p) => p.id === 'plugins')?.category).toBe('system');
    expect(panels.find((p) => p.id === 'ops')?.category).toBe('system');
  });
});

describe('layout persist keys', () => {
  it('uses versioned localStorage names', () => {
    expect('orbisview-layout-v11').toMatch(/^orbisview-layout-v\d+$/);
    expect('orbisview-layers-v8').toMatch(/^orbisview-layers-v\d+$/);
    expect('orbisview-static-slam-v1').toMatch(/^orbisview-static-slam-v\d+$/);
    expect('orbisview-indoor-map-v1').toMatch(/^orbisview-indoor-map-v\d+$/);
    expect('orbisview-annotations-v1').toMatch(/^orbisview-annotations-v\d+$/);
  });

  it('uses versioned view3d opts key', () => {
    expect('orbisview-view3d-opts-v1').toMatch(/^orbisview-view3d-opts-v\d+$/);
  });

  it('uses versioned map view key', () => {
    expect('orbisview-map-view-v2').toMatch(/^orbisview-map-view-v\d+$/);
  });

  it('uses versioned panel opts key', () => {
    expect('orbisview-panel-opts-v1').toMatch(/^orbisview-panel-opts-v\d+$/);
  });
});
