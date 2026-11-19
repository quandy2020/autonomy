# google_robot

Upstream: https://github.com/haosulab/ManiSkill-GoogleRobot

Revision: `3aa86c751ba508acb1272f73dbbd827cd25337b0`

URDF: `google_robot_meta_sim_fix_fingertip.urdf`

Mesh paths were normalized; geometry and joint definitions are upstream assets.

The upstream snapshot contains no LICENSE file; no new license is assigned to these assets.

## Appearance

`meshes/habitat` holds the Habitat-ready visuals, matched to the Everyday Robots
photo set: off-white shells (236, 236, 236), gold gripper fingers (219, 176, 61)
and dark tyres (26, 26, 28). Vents, lettering and arm rings keep their original
texture detail; the torso's gold service panel is recoloured to the white shell.

Every material carries `KHR_materials_unlit`. Habitat-Sim 0.3.3 samples glTF
base-colour textures without an sRGB decode, and its default four-light rig tints
lit shells yellow; unlit materials keep the rendered colour equal to the authored
palette.

`head_sensor_panel.glb` is modelled geometry layered on the upstream head shell:
a black face plate, five lenses with silver rims (a chrome ring on the centre
lens) and two status LEDs, following the head close-up in the reference set. It
is a second `<visual>` on `link_head_tilt`, so it follows head pan and tilt.

Regenerate with trimesh and Pillow installed:

```bash
python scripts/prepare_google_visuals.py --source /path/to/ManiSkill-GoogleRobot
```

`appearance.json` records the palette, the unlit decision and per-file triangle
counts.

## Sensors

The URDF keeps the sensor frames the real robot carries:

| Frame | Use |
| --- | --- |
| `camera_center_optical_frame` | centre RGB-D |
| `camera_left_optical_frame` | stereo left |
| `camera_right_optical_frame` | stereo right |
| `imu.base`, `imu.neck` | IMU |
| `time_of_flight.*` | eight ToF rangefinders around the base |
| `cliff.rear_left`, `cliff.rear_right` | rear cliff sensors |

`config/robots/google_robot.yaml` publishes the centre RGB-D camera plus a
640x480, 60 deg, 0.124 m baseline stereo pair at 15 Hz on
`/camera/left/image_raw`, `/camera/right/image_raw` and the matching
`camera_info` topics. All three ride the head pan and tilt joints.

`/imu/data` runs at 50 Hz in `imu.base`: the orientation is the planar heading
quaternion and linear acceleration carries gravity on `z`.

The eight ToF emitters and two rear cliff sensors publish `sensor_msgs/Range`
at 10 Hz. Habitat-Sim ships no range sensor, so each frame casts one ray along
its URDF `+x` axis and the robot's own collision hull is skipped. ToF beams use
a 4 m budget, cliff beams 0.5 m; a miss reports `inf`, the ROS convention for
"no return". `scripts/smoke_robot_sensors.py --scene <glb>` checks every beam
alongside the cameras and the IMU.
