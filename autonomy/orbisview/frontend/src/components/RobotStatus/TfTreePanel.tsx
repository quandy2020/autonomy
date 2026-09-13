import { useMemo } from 'react';
import { useDataStore } from '@/store/dataStore';
import { SCHEMAS } from '@/store/websocket/types';

interface TfXform {
  parent: string;
  child: string;
  x: number;
  y: number;
  yaw?: number;
}

interface TfTree {
  transforms: TfXform[];
}

interface TreeNode {
  frame: string;
  edge?: TfXform;
  children: TreeNode[];
}

function asPayload<T>(env: { payload?: unknown } | undefined): T | null {
  if (!env?.payload || typeof env.payload !== 'object') return null;
  return env.payload as T;
}

function formatPose(t: TfXform): string {
  return `Δx ${t.x.toFixed(2)}  Δy ${t.y.toFixed(2)}  Δyaw ${(t.yaw ?? 0).toFixed(2)}`;
}

/** Build forest from parent→child edges (rqt_tf_tree style). */
export function buildTfForest(transforms: TfXform[]): {
  roots: TreeNode[];
  cycles: string[];
  orphans: TfXform[];
} {
  const childrenOf = new Map<string, TfXform[]>();
  const childFrames = new Set<string>();
  const allFrames = new Set<string>();

  for (const t of transforms) {
    if (!t.parent || !t.child || t.parent === t.child) continue;
    allFrames.add(t.parent);
    allFrames.add(t.child);
    childFrames.add(t.child);
    const list = childrenOf.get(t.parent) ?? [];
    list.push(t);
    childrenOf.set(t.parent, list);
  }

  const visiting = new Set<string>();
  const visited = new Set<string>();
  const cycles: string[] = [];

  function walk(frame: string, edge?: TfXform): TreeNode | null {
    if (visiting.has(frame)) {
      cycles.push(frame);
      return null;
    }
    if (visited.has(frame) && !edge) {
      // Shared subtree already attached under another root path — skip duplicate root walk.
    }
    visiting.add(frame);
    const kids = (childrenOf.get(frame) ?? [])
      .map((e) => walk(e.child, e))
      .filter((n): n is TreeNode => n != null);
    visiting.delete(frame);
    visited.add(frame);
    return { frame, edge, children: kids };
  }

  const rootNames = [...allFrames].filter((f) => !childFrames.has(f)).sort();
  const roots = rootNames.map((r) => walk(r)).filter((n): n is TreeNode => n != null);

  // Edges whose child was never reached (disconnected / multi-parent conflict).
  const attachedChildren = new Set<string>();
  function collect(n: TreeNode) {
    if (n.edge) attachedChildren.add(n.edge.child);
    n.children.forEach(collect);
  }
  roots.forEach(collect);
  const orphans = transforms.filter(
    (t) => t.child && t.parent && t.parent !== t.child && !attachedChildren.has(t.child),
  );

  return { roots, cycles: [...new Set(cycles)], orphans };
}

function TfSubtree({ node, isLast }: { node: TreeNode; isLast: boolean }) {
  const hasKids = node.children.length > 0;
  return (
    <li className={`tf-tree-item${isLast ? ' is-last' : ''}${hasKids ? ' has-kids' : ''}`}>
      <div className="tf-tree-row">
        <span className="tf-tree-frame" title={node.frame}>
          {node.frame}
        </span>
        {node.edge ? (
          <span className="tf-tree-pose muted" title="relative to parent">
            {formatPose(node.edge)}
          </span>
        ) : (
          <span className="tf-tree-root-tag">root</span>
        )}
      </div>
      {hasKids ? (
        <ul className="tf-tree-children">
          {node.children.map((c, i) => (
            <TfSubtree key={`${c.frame}-${i}`} node={c} isLast={i === node.children.length - 1} />
          ))}
        </ul>
      ) : null}
    </li>
  );
}

export function TfTreePanel() {
  const envelopes = useDataStore((s) => s.envelopes);
  const tf = useMemo(() => {
    const e = Object.values(envelopes).find((x) => x.schema === SCHEMAS.TfTree);
    return asPayload<TfTree>(e);
  }, [envelopes]);

  const list = tf?.transforms ?? [];
  const { roots, cycles, orphans } = useMemo(() => buildTfForest(list), [list]);

  return (
    <div className="panel tf-tree-panel">
      <h3 style={{ marginTop: 0 }}>TF Tree</h3>
      {list.length === 0 ? (
        <p className="muted">no transforms</p>
      ) : (
        <>
          <div className="tf-tree-forest">
            {roots.map((r) => (
              <ul key={r.frame} className="tf-tree-root">
                <TfSubtree node={r} isLast />
              </ul>
            ))}
          </div>
          {cycles.length > 0 ? (
            <p className="warn tf-tree-note">cycle detected: {cycles.join(', ')}</p>
          ) : null}
          {orphans.length > 0 ? (
            <div className="tf-tree-orphans">
              <div className="muted">unattached edges</div>
              <ul className="tf-list">
                {orphans.map((t, i) => (
                  <li key={`${t.parent}-${t.child}-${i}`}>
                    <code>
                      {t.parent} → {t.child}
                    </code>
                    <span className="muted"> {formatPose(t)}</span>
                  </li>
                ))}
              </ul>
            </div>
          ) : null}
        </>
      )}
    </div>
  );
}
