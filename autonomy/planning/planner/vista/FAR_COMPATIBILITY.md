# Vista planner and FAR planner compatibility

Reference implementation: `ros2/autonomous_exploration_development_environment/src/far_planner`
(`contour_detector`, `dynamic_graph`, `contour_graph`, `graph_planner`).

Vista keeps its own contracts (immutable map views, revisioned graph, separate
execution safety) and adopts the FAR planner behaviours that matter for
exploration routing.

| FAR planner concept | Vista implementation |
| --- | --- |
| `NodeFreeDirect` CONVEX / CONCAVE / PILLAR | `ContourDirect` on `VisibilityNode`, derived from the blocked quadrants around each contour cell |
| `CTNode` polygon identity | `VisibilityNode::contour_id`, the minimum cell index of the 4-connected occupied component |
| `NavNode::is_frontier` / boundary handling | `VisibilityNode::frontier` (touches unmapped space) and `boundary` (mapped-area edge) |
| `edge_votes` / `contour_votes` hysteresis | `VisibilityEdge::confirmations` / `rejections` plus `edge_confirmation_votes` |
| attempted paths for unreachable goals | `VistaRoute::attempted` / `attempted_distance`, path ends at the reachable node closest to the goal |
| RViz graph decoder colours | `vista_nodes_convex` / `_concave` / `_pillar` / `_frontier` markers, attempted route drawn in orange |

## Behaviour

* Contour vertices are generated at convex corners, concave corners, pillars,
  coarse contour samples, frontier cells and mapped-area boundary cells.
* Existing vertices keep their identity across updates by world position; their
  class, contour identity and flags are refreshed every revision.
* An edge becomes usable after `edge_confirmation_votes` consecutive
  geometrically valid observations. `1` reproduces immediate admission; higher
  values suppress flickering depth returns at the cost of a slower graph.
  Pending edges are re-checked on the next map revision.
* Unknown cells are not obstacles for contour classification. Routing through
  them still requires `allow_unknown` and always remains a 2D route model: the
  command gate owns real motion admission.
* When neither the visibility graph nor the grid fallback can reach the goal,
  the planner returns the attempted approach instead of an empty result. It is
  reported as `attempted`, never as `success`.

## Configuration

`autonomy/planning/proto/vista_planner.proto`:

```proto
optional uint32 edge_confirmation_votes = 8;  // 1..32, default 1
```

The native RGB-D platform forwards an optional YAML key of the same name:

```yaml
vista_edge_confirmation_votes: 2   # 0 or absent keeps the planner default
```

The validation harness exposes it as --vista-edge-votes.
