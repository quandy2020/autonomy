# Argus testdata

## Real KITTI slice (`kitti_real/`)

Reader: `apps/datasets/kitti_dataset.*` (`argus::datasets`), not libargus.

Do **not** commit synthetic clouds. Use official KITTI:

1. Download [KITTI odometry](https://www.cvlibs.net/datasets/kitti/eval_odometry.php)
   (calibration + velodyne + color) or a raw synced drive.
2. Slice a few frames with C++ (copies only, no synthesis):

```bash
slice_kitti --src /path/to/kitti/sequences/00 \
            --dst testdata/kitti_real --frames 3 --start 0
verify_kitti testdata/kitti_real
offline_kitti --kitti testdata/kitti_real --filter-only
```

Or point tools at a full sequence / raw sync dir:

```bash
export ARGUS_KITTI_ROOT=/path/to/kitti/sequences/00
verify_kitti
```

Raw sync example:

```bash
slice_kitti --src /path/to/2011_09_26/2011_09_26_drive_0005_sync \
            --dst testdata/kitti_real --frames 3
```

KITTI license: http://www.cvlibs.net/datasets/kitti/ — cite Geiger et al.

## Oxford Spires / Newer College

Readers (not in libargus):

- `apps/datasets/oxford_spires_dataset.*` — `raw/lidar-clouds/*.pcd` + `cam0|1|2/*.jpg`
- `apps/datasets/newer_college_dataset.*` — EuRoC-style PCD + `cam*` / `infra1`

```bash
verify_oxford_datasets --spires /path/to/sequences/<name>
verify_oxford_datasets --ncd /path/to/raw_experiment
```

Do not commit clouds/images; point `ARGUS_SPIRES_ROOT` / `ARGUS_NCD_ROOT` at official downloads.
