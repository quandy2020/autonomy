# Autosim robot assets

Each robot is an independent directory with a primary URDF and its meshes:

```text
robot/<profile>/<profile>.urdf
robot/<profile>/meshes/
robot/<profile>/source.json
```

Existing models: TurtleBot3 Burger and Husky. New articulated models: Google Robot and Unitree G1 humanoid (29 DOF) and Go2 quadruped (12 DOF). See `docs/15_robot_articulation.md` for the full profile list and usage.

URDF mesh paths are relative to the robot directory. Imported assets retain their upstream geometry, actuated joint definitions, source revision and licenses. `source.json` records file hashes and upstream paths.

```bash
python -m autosim.robot.viewer --profile google_robot
python -m autosim.robot.viewer --profile unitree_g1
python -m autosim.robot.viewer --profile unitree_go2
```

The Habitat viewer exposes joint sliders and numeric inputs, orbit camera controls, reset and PNG export. Bridge configurations are in `config/robots/<profile>.yaml`. `/joint_commands` accepts named JointState position commands and `/joint_states` reports the current pose. These are kinematic joint controls; locomotion policies are a separate integration.

Unitree display meshes are converted to GLB and simplified to up to 5000 faces per geometry. Geometry transforms and joint definitions are retained; texture UVs use nearest original vertices. Original and display hashes and face counts are recorded in source.json. Full resolution assets can be downloaded using scripts/import_robot_assets.py. Google meshes remain unchanged.
