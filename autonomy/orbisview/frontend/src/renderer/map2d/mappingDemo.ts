import type { StaticSlamBasemap } from './staticSlam';

export type MappingBasemapMeta = {
  originX: number;
  originY: number;
  resolution: number;
  widthPx: number;
  heightPx: number;
};

function basename(url: string): string {
  try {
    const u = url.includes('://') ? new URL(url) : null;
    const path = u?.pathname ?? url;
    const i = path.lastIndexOf('/');
    return i >= 0 ? path.slice(i + 1) : path;
  } catch {
    const i = url.lastIndexOf('/');
    return i >= 0 ? url.slice(i + 1) : url;
  }
}

function naturalNameCompare(a: string, b: string): number {
  return basename(a).localeCompare(basename(b), undefined, {
    numeric: true,
    sensitivity: 'base',
  });
}

/** Natural sort by basename so stage_2.png < stage_10.png */
export function sortMappingFrameUrls(urls: string[]): string[] {
  return [...urls].sort(naturalNameCompare);
}

export function sortMappingFrameFiles(files: File[]): File[] {
  return [...files].sort((a, b) =>
    a.name.localeCompare(b.name, undefined, { numeric: true, sensitivity: 'base' }),
  );
}

/**
 * Build StaticSlamBasemap for one demo frame (source: 'file' | 'url').
 * Does not call store — caller sets via setBasemap.
 */
export function basemapFromDemoFrame(args: {
  imageSrc: string;
  meta: MappingBasemapMeta;
  label?: string;
  source?: 'file' | 'url';
}): StaticSlamBasemap {
  const { imageSrc, meta, label, source = 'file' } = args;
  return {
    imageSrc,
    originX: meta.originX,
    originY: meta.originY,
    resolution: meta.resolution,
    widthPx: meta.widthPx,
    heightPx: meta.heightPx,
    label,
    source,
  };
}
