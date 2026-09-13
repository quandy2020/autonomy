import type { ComponentType } from 'react';
import type { PanelId } from '@/store/layoutStore';

export interface PanelMeta {
  id: PanelId;
  title: string;
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
