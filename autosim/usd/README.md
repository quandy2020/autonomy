# autosim USD assets

Placeholder USD stages for the Isaac Sim backend (`habitat.backend: isaac`).

| Path | Role |
|------|------|
| `worlds/simple_room/simple_room.usd` | Minimal lit room (smoke) |
| `worlds/InteriorAgent/kujiale_*/` | SpatialVerse / 酷家乐 InteriorAgent scenes (default in `isaac.yaml`) |
| `robots/turtlebot3_burger/` | TurtleBot3 Burger robot asset (replace with Isaac / NVIDIA asset) |
| `robots/husky/` | Clearpath Husky robot asset (replace with Isaac / NVIDIA asset) |

Import richer scenes from the Isaac Sim asset library and point `habitat.path`
(or `habitat.usd.world`) at the `.usd` file. Paths in YAML are relative to the
`autosim/` project root.
