import type { ComponentType } from 'react';
import type { PanelId } from '@/store/layoutStore';
import type { IconName } from '@/components/icons';
import { panelBaseId } from '@/components/panelId';

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

export interface PanelProps {
  panelId: string;
}

export interface PanelMeta {
  id: PanelId;
  title: string;
  category: PanelCategory;
  /** Catalog may open multiple mosaic leaves (ids like image#2). */
  allowMultiple?: boolean;
  component: ComponentType<PanelProps>;
}

const registry = new Map<PanelId, PanelMeta>();

export function registerPanel(
  meta: Omit<PanelMeta, 'component'> & { component: ComponentType<PanelProps> | ComponentType },
): void {
  registry.set(meta.id, meta as PanelMeta);
}

export function getPanel(id: PanelId): PanelMeta | undefined {
  return registry.get(panelBaseId(id)) ?? registry.get(id);
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
