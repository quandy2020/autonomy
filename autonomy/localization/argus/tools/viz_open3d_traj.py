#!/usr/bin/env python3
"""Open3D visualization of Argus trajectories (+ optional map.xyz).

Usage:
  python3 scripts/viz_open3d_traj.py \\
    --seq-dir ~/Downloads/oxford_spires_dataset/sequences/2024-07-09-new-college-01 \\
    --modes livo,lio,lo --map .../lio/map/map.xyz --voxel 0.2
"""

from __future__ import annotations

import argparse
import importlib.util
import sys
from pathlib import Path

import numpy as np

try:
    import open3d as o3d
except ImportError as e:
    raise SystemExit(
        "need open3d:  pip install open3d   (or conda install -c open3d-admin open3d)\n"
        f"import error: {e}"
    ) from e

_spec = importlib.util.spec_from_file_location(
    "plot_traj_ate",
    Path(__file__).resolve().parent / "plot_traj_ate.py",
)
_mod = importlib.util.module_from_spec(_spec)
assert _spec.loader is not None
_spec.loader.exec_module(_mod)
load_tum = _mod.load_tum
associate = _mod.associate
umeyama_se3 = _mod.umeyama_se3

COLORS = {
    "gt": (0.05, 0.05, 0.05),
    "livo": (0.89, 0.10, 0.11),
    "lio": (0.22, 0.49, 0.72),
    "lo": (0.60, 0.31, 0.64),
}


def align_xyz(t_est, xyz_est, t_gt, xyz_gt, max_dt: float):
    te, pe, pg = associate(t_est, xyz_est, t_gt, xyz_gt, max_dt)
    if len(te) < 3:
        return None, None
    R, t = umeyama_se3(pe, pg)
    xyz_a = (R @ xyz_est.T).T + t
    err = np.linalg.norm((R @ pe.T).T + t - pg, axis=1)
    rmse = float(np.sqrt(np.mean(err**2)))
    return xyz_a, rmse


def traj_lineset(xyz: np.ndarray, color, name: str) -> o3d.geometry.LineSet:
    pts = xyz.astype(np.float64)
    n = pts.shape[0]
    lines = [[i, i + 1] for i in range(n - 1)]
    ls = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(pts),
        lines=o3d.utility.Vector2iVector(lines),
    )
    ls.colors = o3d.utility.Vector3dVector([color] * len(lines))
    ls_name = name  # for print only; LineSet has no name in older o3d
    _ = ls_name
    return ls


def load_xyz_map(path: Path, voxel: float) -> o3d.geometry.PointCloud:
    # map.xyz: plain "x y z" per line
    pts = []
    with path.open() as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            p = line.split()
            if len(p) < 3:
                continue
            pts.append((float(p[0]), float(p[1]), float(p[2])))
    cloud = o3d.geometry.PointCloud()
    if not pts:
        return cloud
    cloud.points = o3d.utility.Vector3dVector(np.asarray(pts, dtype=np.float64))
    if voxel > 0:
        cloud = cloud.voxel_down_sample(voxel)
    # height coloring
    xyz = np.asarray(cloud.points)
    if len(xyz):
        z = xyz[:, 2]
        zmin, zmax = float(z.min()), float(z.max())
        t = (z - zmin) / (zmax - zmin + 1e-9)
        colors = np.stack([t, 0.35 + 0.4 * (1 - t), 1.0 - t], axis=1)
        cloud.colors = o3d.utility.Vector3dVector(colors)
    return cloud


def transform_cloud(cloud: o3d.geometry.PointCloud, R: np.ndarray, t: np.ndarray):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    cloud.transform(T)
    return cloud


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seq-dir", required=True, help="sequence root")
    ap.add_argument("--modes", default="livo,lio,lo")
    ap.add_argument("--gt", default="", help="override GT tum path")
    ap.add_argument("--max-dt", type=float, default=1.0)
    ap.add_argument("--max-rmse", type=float, default=50.0,
                    help="skip drawing modes with ATE RMSE above this")
    ap.add_argument("--map", default="", help="optional map.xyz to overlay")
    ap.add_argument("--voxel", type=float, default=0.25, help="map downsample")
    ap.add_argument("--align-map-to-gt", action="store_true",
                    help="apply LIO/ first mode Umeyama to map cloud")
    ap.add_argument("--screenshot", default="", help="save image and exit")
    ap.add_argument("--point-size", type=float, default=2.0)
    args = ap.parse_args()

    seq = Path(args.seq_dir).expanduser().resolve()
    gt_path = Path(args.gt) if args.gt else seq / "processed" / "vilens-slam" / "gt.tum"
    if not gt_path.is_file():
        raise SystemExit(f"missing GT: {gt_path}")

    t_gt, xyz_gt = load_tum(gt_path)
    geoms: list = []
    geoms.append(traj_lineset(xyz_gt, COLORS["gt"], "GT"))
    print(f"GT poses={len(t_gt)}  {gt_path}")

    # origin marker
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=2.0)
    geoms.append(axis)

    modes = [m.strip().lower() for m in args.modes.split(",") if m.strip()]
    first_align = None  # (R, t) from first successful mode for map
    for mode in modes:
        est = seq / "argus_eval" / "modes" / mode / "estimate.tum"
        if not est.is_file():
            print(f"{mode}: missing {est}")
            continue
        t_est, xyz_est = load_tum(est)
        xyz_a, rmse = align_xyz(t_est, xyz_est, t_gt, xyz_gt, args.max_dt)
        if xyz_a is None:
            print(f"{mode}: too few associations")
            continue
        print(f"{mode}: poses={len(t_est)} ATE_RMSE={rmse:.3f} m")
        if rmse > args.max_rmse:
            print(f"  skip draw (rmse > {args.max_rmse})")
            continue
        geoms.append(traj_lineset(xyz_a, COLORS.get(mode, (0.5, 0.5, 0.5)), mode.upper()))
        if first_align is None:
            te, pe, pg = associate(t_est, xyz_est, t_gt, xyz_gt, args.max_dt)
            R, t = umeyama_se3(pe, pg)
            first_align = (R, t)

    map_path = Path(args.map).expanduser() if args.map else None
    if map_path and map_path.is_file():
        print(f"load map {map_path} voxel={args.voxel}")
        cloud = load_xyz_map(map_path, args.voxel)
        if args.align_map_to_gt and first_align is not None:
            cloud = transform_cloud(cloud, first_align[0], first_align[1])
        print(f"  cloud points={len(cloud.points)}")
        geoms.append(cloud)
    elif args.map:
        print(f"warn: map not found: {args.map}")

    if args.screenshot:
        # Offscreen if available
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name=f"Argus {seq.name}", width=1600, height=1000, visible=True)
        for g in geoms:
            vis.add_geometry(g)
        opt = vis.get_render_option()
        opt.background_color = np.array([1.0, 1.0, 1.0])
        opt.point_size = args.point_size
        opt.line_width = 3.0
        vis.poll_events()
        vis.update_renderer()
        # better default view
        ctr = vis.get_view_control()
        ctr.set_zoom(0.55)
        out = Path(args.screenshot).expanduser()
        out.parent.mkdir(parents=True, exist_ok=True)
        vis.capture_screen_image(str(out), do_render=True)
        vis.destroy_window()
        print(f"wrote {out}")
        return

    print("Open3D window: mouse orbit / wheel zoom / Q quit")
    o3d.visualization.draw_geometries(
        geoms,
        window_name=f"Argus traj — {seq.name}",
        width=1600,
        height=1000,
        point_show_normal=False,
    )


if __name__ == "__main__":
    main()
