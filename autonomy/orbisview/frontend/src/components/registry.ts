import type { ComponentType } from 'react';
import type { PanelId } from '@/store/layoutStore';
import type { IconName } from '@/components/icons';

/** Catalog grouping for Add Panel. */
export type PanelCategory =
  | 'viz'
  | 'sensor'
  | 'robot'
  | 'planning'
  | 'monitor'
  | 'system';

export const PANEL_CATEGORIES: {
  id: PanelCategory;
  label: string;
  icon: IconName;
}[] = [
  { id: 'viz', label: '可视化', icon: 'view3d' },
  { id: 'sensor', label: '传感器', icon: 'laser' },
  { id: 'robot', label: '机器人', icon: 'robot' },
  { id: 'planning', label: '规划任务', icon: 'route' },
  { id: 'monitor', label: '监控', icon: 'diagnostics' },
  { id: 'system', label: '系统', icon: 'ops' },
];

export interface PanelMeta {
  id: PanelId;
  title: string;
  category: PanelCategory;
  component: ComponentType;
}

const registry = new Map<PanelId, PanelMeta>();

export function registerPanel(meta: PanelMeta): void {
  registry.set(meta.id, meta);
}

export function getPanel(id: PanelId): PanelMeta | undefined {
  return registry.get(id);
}

export function listPanels(): PanelMeta[] {
  return Array.from(registry.values());
}

export function listPanelsByCategory(): {
  category: (typeof PANEL_CATEGORIES)[number];
  panels: PanelMeta[];
}[] {
  const all = listPanels();
  return PANEL_CATEGORIES.map((category) => ({
    category,
    panels: all.filter((p) => p.category === category.id),
  })).filter((g) => g.panels.length > 0);
}
