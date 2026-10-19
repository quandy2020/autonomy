# BENI (Open Beni)

Wheeled-biped balance robot assets ported from
[open-beni](https://github.com/open-beni/open-beni) (`assets/beni_base`, MIT).

| Path | Contents |
|------|----------|
| `beni.urdf` | Kinematics + visuals; autosim adds `base_footprint`, `camera_link`, `imu_link`, `laser_link` |
| `meshes/*.obj` | Visual meshes (Y-up OBJ, standing pose baked into visual origins) |
| `textures/beni_white.png` | Default skin (optional for Autoviz) |

## Motion in autosim

Enable the Open Beni ONNX balancer:

```yaml
habitat:
  policy:
    enabled: true
    action_space: beni
    rate_hz: 100.0
```

- Model: `models/policy.onnx` (from Open Beni `assets/models/policy.onnx`)
- `/cmd_vel` → velocity command in obs `[vx,0,wz]`
- ONNX → 7 actions (L1,R1,L2,R2,LW,RW,head_yaw) → kinematic joint track + TF
- Planar plant still integrates for odom/map (set `drive_from_wheels: true` to
  push base twist from LW/RW)

Full MuJoCo contact dynamics stay in Open Beni; autosim is kinematic + RGB-D.

## Action-sequence tests

```bash
# Pytest (forward / reverse / turn / stop-go / height band)
cd src/autonomy/autosim && python3 -m pytest tests/test_beni_sequences.py -q

# Offline ONNX smoke (prints PASS/FAIL per maneuver)
python3 scripts/beni_action_sequences.py

# Live: publish sequences into a running autosim (Autoviz watches /cmd_vel)
python3 scripts/beni_action_sequences.py --live
python3 scripts/beni_action_sequences.py --live --seq forward --seq turn_left
```

## Height & footprint

| 项 | 值 |
|----|-----|
| `base_link_height` / URDF `base_joint` z | **0.11 m**（Open Beni `H_REF`） |
| `policy.base_height` | **0.11 m**（观测 pose 命令，训练带约 `[0.10, 0.12]`） |
| `policy.animate_joints` | **false**（默认）：TF 不推髋/膝/轮，保持 mesh 烤好的站立姿态；无 MuJoCo 时开 true 会乱蹬 |
| `/footprint` | 显式四点包住双轮（轨距≈0.16），不用机身 collision 小盒 |

## RGB-D

Primary camera is RGB-D (`/camera/rgb/image_raw` + `/camera/depth/image_raw`).
Extrinsics follow MJCF `depth_front` on `base_link`.
