import { useEffect, useMemo, useState } from 'react';
import { useDataStore } from '@/store/dataStore';
import { wsClient } from '@/store/websocket/client';
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

export interface TreeNode {
  frame: string;
  edge?: TfXform;
  children: TreeNode[];
}

interface LaidOut {
  frame: string;
  x: number;
  y: number;
  edge?: TfXform;
  children: LaidOut[];
}

const NODE_RX = 58;
const NODE_RY = 22;
const H_GAP = 36;
const V_GAP = 110;
const PAD = 28;

function asPayload<T>(env: { payload?: unknown } | undefined): T | null {
  if (!env?.payload || typeof env.payload !== 'object') return null;
  return env.payload as T;
}

/** Build forest from parent→child edges. */
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
  const cycles: string[] = [];

  function walk(frame: string, edge?: TfXform): TreeNode | null {
    if (visiting.has(frame)) {
      cycles.push(frame);
      return null;
    }
    visiting.add(frame);
    const kids = (childrenOf.get(frame) ?? [])
      .map((e) => walk(e.child, e))
      .filter((n): n is TreeNode => n != null);
    visiting.delete(frame);
    return { frame, edge, children: kids };
  }

  const rootNames = [...allFrames].filter((f) => !childFrames.has(f)).sort();
  const roots = rootNames.map((r) => walk(r)).filter((n): n is TreeNode => n != null);

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

function shiftTree(node: LaidOut, dx: number): LaidOut {
  return {
    ...node,
    x: node.x + dx,
    children: node.children.map((c) => shiftTree(c, dx)),
  };
}

function layoutNode(node: TreeNode, depth: number): { laid: LaidOut; width: number } {
  if (node.children.length === 0) {
    return {
      laid: {
        frame: node.frame,
        x: 0,
        y: depth * V_GAP,
        edge: node.edge,
        children: [],
      },
      width: NODE_RX * 2,
    };
  }

  const parts = node.children.map((c) => layoutNode(c, depth + 1));
  const totalW =
    parts.reduce((s, p) => s + p.width, 0) + H_GAP * Math.max(0, parts.length - 1);
  let cursor = -totalW / 2;
  const kids: LaidOut[] = [];
  for (const p of parts) {
    const target = cursor + p.width / 2;
    kids.push(shiftTree(p.laid, target - p.laid.x));
    cursor += p.width + H_GAP;
  }

  return {
    laid: {
      frame: node.frame,
      x: 0,
      y: depth * V_GAP,
      edge: node.edge,
      children: kids,
    },
    width: Math.max(NODE_RX * 2, totalW),
  };
}

export function layoutForest(roots: TreeNode[]): {
  nodes: LaidOut[];
  width: number;
  height: number;
} {
  if (roots.length === 0) return { nodes: [], width: 0, height: 0 };
  const parts = roots.map((r) => layoutNode(r, 0));
  const totalW =
    parts.reduce((s, p) => s + p.width, 0) + H_GAP * 2 * Math.max(0, parts.length - 1);
  let cursor = 0;
  const nodes: LaidOut[] = [];
  for (const p of parts) {
    const target = cursor + p.width / 2;
    nodes.push(shiftTree(p.laid, target - p.laid.x));
    cursor += p.width + H_GAP * 2;
  }
  let maxY = 0;
  function walkY(n: LaidOut) {
    maxY = Math.max(maxY, n.y);
    n.children.forEach(walkY);
  }
  nodes.forEach(walkY);
  return {
    nodes,
    width: totalW + PAD * 2,
    height: maxY + NODE_RY * 2 + PAD * 2 + 40,
  };
}

function flattenEdges(nodes: LaidOut[]): { parent: LaidOut; child: LaidOut }[] {
  const edges: { parent: LaidOut; child: LaidOut }[] = [];
  function walk(n: LaidOut) {
    for (const c of n.children) {
      edges.push({ parent: n, child: c });
      walk(c);
    }
  }
  nodes.forEach(walk);
  return edges;
}

function flattenNodes(nodes: LaidOut[]): LaidOut[] {
  const out: LaidOut[] = [];
  function walk(n: LaidOut) {
    out.push(n);
    n.children.forEach(walk);
  }
  nodes.forEach(walk);
  return out;
}

function TfGraph({
  roots,
  avgHz,
}: {
  roots: TreeNode[];
  avgHz: number | null;
}) {
  const { nodes, width, height } = useMemo(() => layoutForest(roots), [roots]);
  const edges = useMemo(() => flattenEdges(nodes), [nodes]);
  const flat = useMemo(() => flattenNodes(nodes), [nodes]);
  const xs = flat.map((n) => n.x);
  const minX = xs.length ? Math.min(...xs) : 0;
  const maxX = xs.length ? Math.max(...xs) : 0;
  const contentW = Math.max(width, maxX - minX + NODE_RX * 2 + PAD * 2);
  const offsetX = PAD + NODE_RX - minX;

  return (
    <div className="tf-graph-scroll">
      <svg
        className="tf-graph-svg"
        width={contentW}
        height={height}
        viewBox={`0 0 ${contentW} ${height}`}
        role="img"
        aria-label="TF frames graph"
      >
        <defs>
          <marker
            id="tf-arrow"
            viewBox="0 0 10 10"
            refX="9"
            refY="5"
            markerWidth="7"
            markerHeight="7"
            orient="auto-start-reverse"
          >
            <path d="M 0 0 L 10 5 L 0 10 z" fill="#9db0c4" />
          </marker>
        </defs>

        <rect
          x={8}
          y={8}
          width={Math.min(220, contentW - 16)}
          height={36}
          rx={4}
          className="tf-graph-banner"
        />
        <text x={18} y={24} className="tf-graph-banner-title">
          view_frames Result
        </text>
        <text x={18} y={38} className="tf-graph-banner-sub">
          OrbisView TF tree
        </text>

        {edges.map(({ parent, child }) => {
          const x1 = parent.x + offsetX;
          const y1 = parent.y + PAD + 48 + NODE_RY;
          const x2 = child.x + offsetX;
          const y2 = child.y + PAD + 48 - NODE_RY;
          const mx = (x1 + x2) / 2;
          const my = (y1 + y2) / 2;
          const e = child.edge;
          const lines = [
            'Broadcaster: mock',
            avgHz != null && Number.isFinite(avgHz)
              ? `Average rate: ${avgHz.toFixed(3)}`
              : 'Average rate: —',
            e
              ? `xyz: ${e.x.toFixed(2)}, ${e.y.toFixed(2)}, 0.00`
              : '',
            e ? `yaw: ${(e.yaw ?? 0).toFixed(3)}` : '',
          ].filter(Boolean);
          return (
            <g key={`${parent.frame}->${child.frame}`}>
              <path
                d={`M ${x1} ${y1} C ${x1} ${(y1 + y2) / 2}, ${x2} ${(y1 + y2) / 2}, ${x2} ${y2}`}
                className="tf-graph-edge"
                markerEnd="url(#tf-arrow)"
              />
              <rect
                x={mx - 78}
                y={my - 28}
                width={156}
                height={14 + lines.length * 11}
                rx={3}
                className="tf-graph-edge-label-bg"
              />
              {lines.map((line, i) => (
                <text
                  key={i}
                  x={mx}
                  y={my - 14 + i * 11}
                  textAnchor="middle"
                  className="tf-graph-edge-label"
                >
                  {line}
                </text>
              ))}
            </g>
          );
        })}

        {flat.map((n) => {
          const cx = n.x + offsetX;
          const cy = n.y + PAD + 48;
          return (
            <g key={n.frame}>
              <ellipse cx={cx} cy={cy} rx={NODE_RX} ry={NODE_RY} className="tf-graph-node" />
              <text x={cx} y={cy + 4} textAnchor="middle" className="tf-graph-node-label">
                {n.frame}
              </text>
            </g>
          );
        })}
      </svg>
    </div>
  );
}

export function TfTreePanel() {
  const envelopes = useDataStore((s) => s.envelopes);
  const connected = useDataStore((s) => s.connected);
  const [tfHz, setTfHz] = useState<number | null>(null);

  const tfEnv = useMemo(() => {
    return Object.values(envelopes).find((x) => x.schema === SCHEMAS.TfTree);
  }, [envelopes]);

  const tf = asPayload<TfTree>(tfEnv);
  const list = tf?.transforms ?? [];
  const { roots, cycles, orphans } = useMemo(() => buildTfForest(list), [list]);

  useEffect(() => {
    if (!connected) {
      setTfHz(null);
      return;
    }
    const off = wsClient.onMessage((msg) => {
      if (msg.op !== 'channel_stats') return;
      const row = (msg.channels ?? []).find(
        (c) => c.name.includes('/tf') || c.name.endsWith('tf'),
      );
      setTfHz(row?.hz ?? null);
    });
    wsClient.send({ op: 'channel_stats' });
    const t = window.setInterval(() => wsClient.send({ op: 'channel_stats' }), 1000);
    return () => {
      off();
      window.clearInterval(t);
    };
  }, [connected]);

  return (
    <div className="panel tf-tree-panel">
      <h3 style={{ marginTop: 0 }}>TF Tree</h3>
      {list.length === 0 ? (
        <p className="muted">no transforms</p>
      ) : (
        <>
          <TfGraph roots={roots} avgHz={tfHz} />
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
                    <span className="muted">
                      {' '}
                      Δx {t.x.toFixed(2)} Δy {t.y.toFixed(2)} Δyaw {(t.yaw ?? 0).toFixed(2)}
                    </span>
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
