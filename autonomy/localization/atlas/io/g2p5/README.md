# G2P5 — 2.5D lidar occupancy (Atlas `io/g2p5`)

Port of lightning-lm `core/g2p5`, adapted for Atlas types. **Display / nav
export only** — not used for localization or dual-system fusion.

## Layout

| File | Role |
|------|------|
| `g2p5_grid_data.hpp` | Per-cell hit/visit/height |
| `g2p5_subgrid.*` | Lazy 16×16 sub-grid |
| `g2p5_map.*` | Hierarchical map; `ToCV()` / `ToROS()` → OccupancyGrid |
| `g2p5.*` | Frontend push + backend redraw |
| `g2p5_projector.*` | Facade / one-shot `Project()` for callers |

Namespace: `autonomy::localization::atlas::map`.

## Keyframe

```cpp
struct G2P5Keyframe {
  std::uint64_t id = 0;
  Mat44_t T_wb = Mat44_t::Identity();
  std::vector<Vec3_t> points_body;  // lidar body
};
```

`LidarBridge` builds these on `DecideAndPush` and calls `PushKeyframe`.
After lidar pose-graph `Optimize` success → `RedrawGlobalMap()`.

## Wiring

- `RuntimeConfig.maps_g2p5` / profile `maps.g2p5: true`
- `LocalizationServer` creates `map::G2P5`, `Init()`, `LidarBridge::SetG2P5`
- `G2P5Projector` optional facade holding `shared_ptr<G2P5>`

## Notes vs lightning

- No ROS2 `nav_msgs`; `ToROS()` fills `automsgs::msgs::map_msgs::OccupancyGrid`
- No `Keyframe::Ptr` / `AsyncMessageProcess` / Timer — `std::thread` + queue
- Floor: optional PCL `SACSegmentation`; default plane when `esti_floor_=false`
