# autosim USD assets

Placeholder USD stages for the Isaac Sim backend (`habitat.backend: isaac`).

| Path | Role |
|------|------|
| `worlds/simple_room/simple_room.usd` | Minimal lit room (smoke) |
| `worlds/InteriorAgent/kujiale_*/` | SpatialVerse / 酷家乐 InteriorAgent scenes (default in `isaac.yaml`) |
| `robots/turtlebot3_burger/` | TurtleBot3 Burger robot asset (replace with Isaac / NVIDIA asset) |
| `robots/husky/` | Clearpath Husky robot asset (replace with Isaac / NVIDIA asset) |
| `robots/beni/` | Open Beni placeholder Xform (visuals from `robot/beni` URDF in Autoviz) |
| `people/Characters/` | Isaac People 人体 USD（gitignore；供 `pedestrian_visual: character`） |
| `people/Animations/` | walk/idle SkelAnimation USD（可选；`pedestrian_anim`） |

将 Isaac Sim 的 `People/Characters`（及 `Animations`）拷到 `people/`，或设
`character_root` / `animation_root` / `AUTOSIM_PEOPLE_ROOT`。

Import richer scenes from the Isaac Sim asset library and point `habitat.path`
(or `habitat.usd.world`) at the `.usd` file. Paths in YAML are relative to the
`autosim/` project root.
