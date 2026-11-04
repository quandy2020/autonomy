# Habitat SMPL-X humanoids (TrackVLA layout)

Assets live **inside autosim** — no TrackVLA checkout required:

```text
data/humanoids/humanoid_data/<name>/<name>.urdf
data/humanoids/humanoid_data/<name>/<name>_motion_data_smplx.pkl
```

## Download

```bash
cd src/autonomy/autosim
python3 scripts/download_humanoids.py
# or, if Drive blocks CLI:
#   download https://drive.google.com/file/d/1aE_wyvPqvOuVmF8px2vTO3trr70DKf1l/view
#   python3 scripts/download_humanoids.py --zip ~/Downloads/humanoids.zip
```

`habitat.humanoids.root` defaults to `data/humanoids/humanoid_data` (relative to autosim).

See [docs/14_habitat_humanoids.md](../../docs/14_habitat_humanoids.md).
