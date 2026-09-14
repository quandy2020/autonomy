/** SE(2) TF composition for map overlay (parent→child relative → world). */

export interface TfXform {
  parent: string;
  child: string;
  x: number;
  y: number;
  yaw?: number;
}

export interface TfWorldFrame {
  frame: string;
  x: number;
  y: number;
  yaw: number;
  parent?: string;
}

function compose(
  px: number,
  py: number,
  pyaw: number,
  lx: number,
  ly: number,
  lyaw: number,
): { x: number; y: number; yaw: number } {
  const c = Math.cos(pyaw);
  const s = Math.sin(pyaw);
  return {
    x: px + c * lx - s * ly,
    y: py + s * lx + c * ly,
    yaw: pyaw + lyaw,
  };
}

const ROOT_PREF = ['map', 'odom', 'world', 'base_link'];

export function frameByName(
  frames: TfWorldFrame[],
  name: string | null | undefined,
): TfWorldFrame | undefined {
  if (!name) return undefined;
  return frames.find((f) => f.frame === name);
}

/** Prefer map-rooted base / laser poses for overlay alignment with OccupancyGrid. */
export function resolveDrawPose(
  frames: TfWorldFrame[],
  fallback: { x: number; y: number; yaw?: number } | null | undefined,
  prefer: string[] = ['base_link', 'base_footprint'],
): { x: number; y: number; yaw: number } | null {
  for (const name of prefer) {
    const f = frameByName(frames, name);
    if (f) return { x: f.x, y: f.y, yaw: f.yaw };
  }
  if (!fallback) return null;
  return { x: fallback.x, y: fallback.y, yaw: fallback.yaw ?? 0 };
}

export function resolveLaserDrawPose(
  frames: TfWorldFrame[],
  frameId: string | null | undefined,
  fallback: { x: number; y: number; yaw?: number } | null | undefined,
): { x: number; y: number; yaw: number } | null {
  const prefer = [
    frameId,
    'laser_link',
    'base_scan',
    'base_link',
    'base_footprint',
  ].filter((x): x is string => !!x);
  return resolveDrawPose(frames, fallback, prefer);
}

/**
 * Compose parent→child edges into world poses.
 * Roots (no parent edge) sit at identity; prefer map/odom as traversal roots.
 */
export function resolveTfWorldFrames(transforms: TfXform[]): TfWorldFrame[] {
  const byChild = new Map<string, TfXform>();
  const childrenOf = new Map<string, TfXform[]>();
  const childFrames = new Set<string>();
  const allFrames = new Set<string>();

  for (const t of transforms) {
    if (!t?.parent || !t?.child || t.parent === t.child) continue;
    // Last edge for a child wins (dynamic /tf over earlier /tf_static).
    byChild.set(t.child, t);
    allFrames.add(t.parent);
    allFrames.add(t.child);
    childFrames.add(t.child);
  }

  for (const t of byChild.values()) {
    const list = childrenOf.get(t.parent) ?? [];
    list.push(t);
    childrenOf.set(t.parent, list);
  }

  const roots = [...allFrames].filter((f) => !childFrames.has(f));
  roots.sort((a, b) => {
    const ia = ROOT_PREF.indexOf(a);
    const ib = ROOT_PREF.indexOf(b);
    return (ia < 0 ? 99 : ia) - (ib < 0 ? 99 : ib) || a.localeCompare(b);
  });

  const world = new Map<string, TfWorldFrame>();
  const visiting = new Set<string>();

  function place(frame: string, pose: TfWorldFrame): void {
    if (world.has(frame)) return;
    if (visiting.has(frame)) return;
    visiting.add(frame);
    world.set(frame, pose);
    for (const edge of childrenOf.get(frame) ?? []) {
      const yaw = edge.yaw ?? 0;
      const childPose = compose(pose.x, pose.y, pose.yaw, edge.x, edge.y, yaw);
      place(edge.child, {
        frame: edge.child,
        x: childPose.x,
        y: childPose.y,
        yaw: childPose.yaw,
        parent: frame,
      });
    }
    visiting.delete(frame);
  }

  for (const root of roots) {
    place(root, { frame: root, x: 0, y: 0, yaw: 0 });
  }

  // Orphan edges whose parent never got placed (partial tree): still show at relative xy
  // only if parent exists in world; otherwise skip to avoid origin cluster noise.
  for (const t of byChild.values()) {
    if (world.has(t.child)) continue;
    const parent = world.get(t.parent);
    if (!parent) continue;
    const yaw = t.yaw ?? 0;
    const childPose = compose(parent.x, parent.y, parent.yaw, t.x, t.y, yaw);
    world.set(t.child, {
      frame: t.child,
      x: childPose.x,
      y: childPose.y,
      yaw: childPose.yaw,
      parent: t.parent,
    });
  }

  return [...world.values()];
}

/** Merge several TfTree payloads; later trees override same child edge. */
export function mergeTfTransforms(lists: Array<TfXform[] | null | undefined>): TfXform[] {
  const byChild = new Map<string, TfXform>();
  for (const list of lists) {
    if (!list) continue;
    for (const t of list) {
      if (!t?.parent || !t?.child) continue;
      byChild.set(t.child, t);
    }
  }
  return [...byChild.values()];
}
