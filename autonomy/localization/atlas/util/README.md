# Atlas `util/`

Atlas-specific helpers. Prefer **`autonomy/common`** for shared math / strings / angles.

| Keep (Atlas-specific) | Role |
|----------------------|------|
| `calibration/` | multimodal calib YAML |
| `extrinsics.hpp` | fusion extrinsics subset |
| `modality.*` | VO…LVWIO flags |
| `schedule.hpp` / publishers / sqlite / stereo / converter / … | vision SLAM plumbing |
| `trigonometric.hpp` | ORB fast sin/cos tables |
| `constant.hpp` | **only** `kGRAVITY` (common has no gravity constant) |

| Deleted (reuse common) | Use instead |
|------------------------|-------------|
| `angle.hpp/.cpp` | `common/math/angle.hpp` → `AngleDiffDegrees` |
| `yaml.hpp/.cpp` | `common/param_handler.hpp` → `YamlChild` / `ParseRectangles` |
| `so3.hpp` | `common/math/so3.hpp` |
| `string.hpp` | `common/string_util.hpp` |
| `lock` / `timed_pose` / `pose_rpy` / `loop_candidate` | std / `type.hpp` / `LidarLoopResult` |
