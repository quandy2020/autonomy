# TurtleBot3 Burger meshes

Official ROBOTIS visuals (Apache-2.0) from
[`turtlebot3_description`](https://github.com/ROBOTIS-GIT/turtlebot3/tree/humble/turtlebot3_description):

| Path | Part |
|------|------|
| `bases/burger_base.stl` | Chassis |
| `wheels/left_tire.stl` / `right_tire.stl` | Drive wheels |
| `sensors/lds.stl` | LDS-01 lidar |

STL units are millimetres; URDF uses `scale="0.001 0.001 0.001"`.
Paths in the URDF are relative to the parent directory (Autoviz / Assimp resolve via URDF `baseDirectory`).
