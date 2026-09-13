/**
 * Frontend plugin hot-load via manifest JSON.
 * Place manifests under public/plugins/ or serve via document_root.
 *
 * Manifest example:
 * {
 *   "plugins": [
 *     { "id": "demo_fe", "title": "Demo FE", "entry": "/plugins/demo.js" }
 *   ]
 * }
 *
 * Entry scripts may call window.__orbisviewRegisterPanel({ id, title, mount }).
 */

import { registerPanel } from '@/components/registry';
import type { ComponentType } from 'react';
import { createElement, useEffect, useState } from 'react';

export interface FrontendPluginManifest {
  id: string;
  title: string;
  entry?: string;
  version?: string;
}

declare global {
  interface Window {
    __orbisviewRegisterPanel?: (spec: {
      id: string;
      title: string;
      mount: (el: HTMLElement) => void | (() => void);
    }) => void;
  }
}

function ExternalMountPanel({ mount }: { mount: (el: HTMLElement) => void | (() => void) }) {
  const [ref, setRef] = useState<HTMLDivElement | null>(null);
  useEffect(() => {
    if (!ref) return;
    const cleanup = mount(ref);
    return typeof cleanup === 'function' ? cleanup : undefined;
  }, [ref, mount]);
  return createElement('div', { ref: setRef, className: 'panel external-plugin' });
}

function installBridge(): void {
  if (window.__orbisviewRegisterPanel) return;
  window.__orbisviewRegisterPanel = (spec) => {
    const Comp: ComponentType = () =>
      createElement(ExternalMountPanel, { mount: spec.mount });
    registerPanel({
      id: spec.id,
      title: spec.title,
      category: 'system',
      component: Comp,
    });
  };
}

export async function loadFrontendPluginManifests(url: string): Promise<number> {
  installBridge();
  const res = await fetch(url, { cache: 'no-store' });
  if (res.status === 404) return 0;
  if (!res.ok) throw new Error(`manifest fetch failed: ${res.status}`);
  const body = (await res.json()) as { plugins?: FrontendPluginManifest[] };
  const list = body.plugins ?? [];
  let loaded = 0;
  for (const p of list) {
    if (!p.entry || !p.id) continue;
    await new Promise<void>((resolve, reject) => {
      const s = document.createElement('script');
      s.src = p.entry!;
      s.async = true;
      s.onload = () => {
        ++loaded;
        resolve();
      };
      s.onerror = () => reject(new Error(`failed to load ${p.entry}`));
      document.head.appendChild(s);
    }).catch(() => {
      /* isolated failure */
    });
  }
  return loaded;
}
