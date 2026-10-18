#!/usr/bin/env python3
"""RViz-like incremental mapping playback (Open3D).

Replays a saved Argus map package with a dense timeline:
  - map.xyz revealed progressively (nearest keyframe along the path)
  - current keyframe scan (bright orange)
  - dense odometry path + keyframe path + GT (time-synced)
  - moving pose axes

Controls (focus the window):
  Space   pause / resume
  F       toggle follow-camera
  A       auto-fit overview (also turns follow off)
  Q/Esc   quit
  mouse   orbit / zoom / pan

Usage:
  python3 scripts/viz_open3d_mapping.py \\
    --seq-dir ~/Downloads/oxford_spires_dataset/sequences/2024-07-09-new-college-01 \\
    --mode lio --speed 1
"""

from __future__ import annotations

import argparse
import faulthandler
import importlib.util
import time
from pathlib import Path

import numpy as np

faulthandler.enable()

try:
    import open3d as o3d
except ImportError as e:
    raise SystemExit(f"need open3d: pip install open3d\n{e}") from e

_spec = importlib.util.spec_from_file_location(
    "plot_traj_ate",
    Path(__file__).resolve().parent / "plot_traj_ate.py",
)
_mod = importlib.util.module_from_spec(_spec)
assert _spec.loader is not None
_spec.loader.exec_module(_mod)
associate = _mod.associate
umeyama_se3 = _mod.umeyama_se3

COLORS = {
    "gt": (0.75, 0.75, 0.75),
    "path": (0.15, 0.85, 0.30),
    "odom": (0.25, 0.60, 1.00),
    "scan": (1.00, 0.45, 0.10),
    "pose": (0.95, 0.20, 0.20),
}


def load_tum(path: Path):
    rows = []
    with path.open() as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            p = line.split()
            if len(p) < 8:
                continue
            rows.append([float(x) for x in p[:8]])
    a = np.asarray(rows, dtype=np.float64)
    return a[:, 0], a[:, 1:4], a[:, 4:8]


def quat_xyzw_to_R(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = q / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def tum_to_T(xyz: np.ndarray, quat: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = quat_xyzw_to_R(quat)
    T[:3, 3] = xyz
    return T


def height_colors(xyz: np.ndarray) -> np.ndarray:
    if xyz.size == 0:
        return np.zeros((0, 3))
    z = xyz[:, 2]
    lo, hi = np.percentile(z, [5, 95])
    if hi - lo < 1e-6:
        hi = lo + 1.0
    tt = np.clip((z - lo) / (hi - lo), 0, 1)
    return np.stack(
        [0.25 + 0.55 * tt, 0.45 + 0.35 * (1 - tt), 0.75 - 0.45 * tt],
        axis=1,
    )


def set_lineset(ls: o3d.geometry.LineSet, xyz: np.ndarray, color):
    n = xyz.shape[0]
    if n < 2:
        p0 = xyz[0] if n == 1 else np.zeros(3)
        pts = np.vstack([p0, p0])
        ls.points = o3d.utility.Vector3dVector(pts)
        ls.lines = o3d.utility.Vector2iVector([[0, 1]])
        ls.colors = o3d.utility.Vector3dVector([color])
        return
    lines = np.column_stack([np.arange(n - 1), np.arange(1, n)])
    ls.points = o3d.utility.Vector3dVector(xyz)
    ls.lines = o3d.utility.Vector2iVector(lines)
    ls.colors = o3d.utility.Vector3dVector(np.tile(np.asarray(color), (n - 1, 1)))


def set_pose_axes(ls: o3d.geometry.LineSet, T: np.ndarray, size: float = 1.5):
    """RGB axes as LineSet (avoids TriangleMesh recreate crashes)."""
    o = T[:3, 3]
    R = T[:3, :3]
    pts = np.vstack([o, o + size * R[:, 0], o + size * R[:, 1], o + size * R[:, 2]])
    ls.points = o3d.utility.Vector3dVector(pts)
    ls.lines = o3d.utility.Vector2iVector([[0, 1], [0, 2], [0, 3]])
    ls.colors = o3d.utility.Vector3dVector(
        [[1.0, 0.2, 0.2], [0.2, 1.0, 0.2], [0.2, 0.4, 1.0]]
    )


def load_keyframe_bin(path: Path) -> np.ndarray:
    raw = np.fromfile(path, dtype=np.float32)
    if raw.size % 4 != 0:
        raw = raw[: raw.size - (raw.size % 4)]
    return raw.reshape(-1, 4)[:, :3].astype(np.float64)


def recover_keyframe_times(pose_xyz: np.ndarray, traj_t: np.ndarray, est_path: Path):
    if len(traj_t) >= 2 and float(np.ptp(traj_t)) > 1.0:
        return traj_t.astype(np.float64)
    if not est_path.is_file():
        return traj_t[0] + np.arange(len(traj_t), dtype=np.float64) * 0.1
    et, exyz, _ = load_tum(est_path)
    times = np.zeros(len(pose_xyz), dtype=np.float64)
    for i, p in enumerate(pose_xyz):
        d2 = np.sum((exyz - p) ** 2, axis=1)
        times[i] = et[int(np.argmin(d2))]
    for i in range(1, len(times)):
        if times[i] <= times[i - 1]:
            times[i] = times[i - 1] + 1e-3
    return times


def load_keyframes(map_dir: Path, voxel: float):
    traj_path = map_dir / "trajectory.tum"
    idx_path = map_dir / "keyframes" / "index.txt"
    if not traj_path.is_file():
        raise SystemExit(f"missing {traj_path}")
    if not idx_path.is_file():
        raise SystemExit(f"missing {idx_path}")

    t, xyz, quat = load_tum(traj_path)
    t = recover_keyframe_times(xyz, t, map_dir.parent / "estimate.tum")
    by_id = {}
    with idx_path.open() as f:
        for line in f:
            parts = line.split()
            if len(parts) < 4:
                continue
            by_id[int(parts[0])] = (parts[1], int(parts[3]) != 0)

    kfs = []
    kf_dir = map_dir / "keyframes"
    for i in range(len(t)):
        meta = by_id.get(i)
        if meta is None:
            continue
        name, in_world = meta
        bin_path = kf_dir / name
        if not bin_path.is_file():
            continue
        pts = load_keyframe_bin(bin_path)
        if pts.shape[0] == 0:
            continue
        T = tum_to_T(xyz[i], quat[i])
        if in_world:
            pts_w = pts
        else:
            pts_w = (T[:3, :3] @ pts.T).T + T[:3, 3]
        if voxel > 1e-9 and pts_w.shape[0] > 50:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(pts_w)
            pcd = pcd.voxel_down_sample(voxel)
            pts_w = np.asarray(pcd.points)
        kfs.append({"t": float(t[i]), "T": T, "xyz": pts_w, "pose": xyz[i].copy()})
    if not kfs:
        raise SystemExit("no keyframes loaded")
    return kfs


def load_map_xyz(path: Path, voxel: float) -> np.ndarray:
    pts = []
    with path.open() as f:
        for line in f:
            p = line.split()
            if len(p) < 3:
                continue
            pts.append((float(p[0]), float(p[1]), float(p[2])))
    xyz = np.asarray(pts, dtype=np.float64)
    if xyz.size == 0:
        return xyz
    if voxel > 1e-9:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        pcd = pcd.voxel_down_sample(voxel)
        xyz = np.asarray(pcd.points)
    return xyz


def assign_points_to_keyframes(map_xyz: np.ndarray, kf_poses: np.ndarray) -> np.ndarray:
    """Each map point → index of nearest keyframe pose (chunked)."""
    n = map_xyz.shape[0]
    assign = np.zeros(n, dtype=np.int32)
    chunk = 50000
    for s in range(0, n, chunk):
        e = min(n, s + chunk)
        # (chunk, 1, 3) - (1, K, 3)
        d2 = np.sum((map_xyz[s:e, None, :] - kf_poses[None, :, :]) ** 2, axis=2)
        assign[s:e] = np.argmin(d2, axis=1)
    return assign


def umeyama_from_traj(xyz_est, t_est, gt_path: Path, max_dt: float):
    if not gt_path.is_file():
        return None
    t_gt, xyz_gt, _ = load_tum(gt_path)
    te, pe, pg = associate(t_est, xyz_est, t_gt, xyz_gt, max_dt)
    if len(te) < 3:
        return None
    R, t = umeyama_se3(pe, pg)
    err = np.linalg.norm((R @ pe.T).T + t - pg, axis=1)
    rmse = float(np.sqrt(np.mean(err**2)))
    return R, t, rmse, t_gt, xyz_gt


def apply_SE3(xyz: np.ndarray, R: np.ndarray, t: np.ndarray) -> np.ndarray:
    return (R @ xyz.T).T + t


def pose_at_time(t_arr, xyz, quat, t_now):
    j = int(np.searchsorted(t_arr, t_now, side="right") - 1)
    j = max(0, min(len(t_arr) - 1, j))
    return xyz[j], quat[j], j


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seq-dir", default="")
    ap.add_argument("--map-dir", default="")
    ap.add_argument("--mode", default="lio")
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--scan-voxel", type=float, default=0.15)
    ap.add_argument("--map-voxel", type=float, default=0.30, help="full map.xyz backdrop voxel")
    ap.add_argument("--live-voxel", type=float, default=0.20, help="incremental keyframe map voxel")
    ap.add_argument("--odom-stride", type=int, default=1, help="use every N-th estimate pose as timeline")
    ap.add_argument("--follow", action="store_true")
    ap.add_argument("--no-gt", action="store_true")
    ap.add_argument("--no-odom", action="store_true")
    ap.add_argument("--no-full-map", action="store_true", help="hide complete map.xyz backdrop")
    ap.add_argument("--max-dt", type=float, default=1.0)
    ap.add_argument("--loop", action="store_true")
    ap.add_argument("--record-dir", default="")
    ap.add_argument("--record-stride", type=int, default=2)
    args = ap.parse_args()

    if args.map_dir:
        map_dir = Path(args.map_dir).expanduser().resolve()
    elif args.seq_dir:
        seq = Path(args.seq_dir).expanduser().resolve()
        map_dir = seq / "argus_eval" / "modes" / args.mode / "map"
    else:
        raise SystemExit("need --seq-dir or --map-dir")
    if not map_dir.is_dir():
        raise SystemExit(f"map dir missing: {map_dir}")

    print(f"map-dir {map_dir}")
    kfs = load_keyframes(map_dir, args.scan_voxel)
    kf_poses = np.stack([k["pose"] for k in kfs])
    kf_t = np.array([k["t"] for k in kfs])
    print(f"keyframes {len(kfs)}  duration={kf_t[-1]-kf_t[0]:.1f}s")

    # complete map.xyz as static backdrop (never re-uploaded → stable)
    map_path = map_dir / "map.xyz"
    if not map_path.is_file():
        raise SystemExit(f"missing {map_path}")
    print(f"loading complete map.xyz voxel={args.map_voxel} …")
    map_xyz = load_map_xyz(map_path, args.map_voxel)
    print(f"  full map points={len(map_xyz)}")

    # dense odom timeline
    est_path = map_dir.parent / "estimate.tum"
    if not est_path.is_file():
        raise SystemExit(f"missing {est_path} (needed for dense timeline)")
    odom_t, odom_xyz, odom_q = load_tum(est_path)
    stride = max(1, args.odom_stride)
    timeline_t = odom_t[::stride]
    timeline_xyz = odom_xyz[::stride]
    timeline_q = odom_q[::stride]
    print(f"timeline poses={len(timeline_t)} (odom stride={stride})")

    # Umeyama → GT
    R_a, t_a = np.eye(3), np.zeros(3)
    gt_t = gt_xyz_full = None
    gt_path = None
    if args.seq_dir:
        gt_path = Path(args.seq_dir).expanduser().resolve() / "processed" / "vilens-slam" / "gt.tum"
    elif (map_dir.parents[2] / "processed" / "vilens-slam" / "gt.tum").is_file():
        gt_path = map_dir.parents[2] / "processed" / "vilens-slam" / "gt.tum"

    if not args.no_gt and gt_path and gt_path.is_file():
        al = umeyama_from_traj(odom_xyz, odom_t, gt_path, args.max_dt)
        if al is not None:
            R_a, t_a, rmse, gt_t, gt_xyz_full = al
            print(f"Umeyama ATE_RMSE={rmse:.3f} m (odom → GT)")
            map_xyz = apply_SE3(map_xyz, R_a, t_a)
            for k in kfs:
                k["xyz"] = apply_SE3(k["xyz"], R_a, t_a)
                k["pose"] = apply_SE3(k["pose"].reshape(1, 3), R_a, t_a)[0]
                T = np.eye(4)
                T[:3, :3] = R_a @ k["T"][:3, :3]
                T[:3, 3] = R_a @ k["T"][:3, 3] + t_a
                k["T"] = T
            kf_poses = np.stack([k["pose"] for k in kfs])
            timeline_xyz = apply_SE3(timeline_xyz, R_a, t_a)
            odom_xyz = apply_SE3(odom_xyz, R_a, t_a)
            for i in range(len(timeline_q)):
                Ri = R_a @ quat_xyzw_to_R(timeline_q[i])
                tr = np.trace(Ri)
                if tr > 0:
                    s = 0.5 / np.sqrt(tr + 1.0)
                    w = 0.25 / s
                    x = (Ri[2, 1] - Ri[1, 2]) * s
                    y = (Ri[0, 2] - Ri[2, 0]) * s
                    z = (Ri[1, 0] - Ri[0, 1]) * s
                else:
                    w, x, y, z = 1.0, 0.0, 0.0, 0.0
                timeline_q[i] = [x, y, z, w]
        else:
            print("warn: GT associate failed")

    # dim complete map + bright height-colored live accum from keyframes
    map_colors_dim = height_colors(map_xyz) * 0.35 + 0.15

    # --- window ---
    vis = o3d.visualization.VisualizerWithKeyCallback()
    title = f"Argus mapping [{args.mode}] — complete map + live scans  (Space/F/A/Q)"
    vis.create_window(window_name=title, width=1600, height=1000)
    opt = vis.get_render_option()
    opt.background_color = np.array([0.08, 0.08, 0.10])
    opt.point_size = 2.0
    opt.line_width = 4.0

    # static full map (uploaded once)
    if not args.no_full_map and len(map_xyz) > 0:
        full_pcd = o3d.geometry.PointCloud()
        full_pcd.points = o3d.utility.Vector3dVector(map_xyz)
        full_pcd.colors = o3d.utility.Vector3dVector(map_colors_dim)
        vis.add_geometry(full_pcd)
        print(f"  backdrop: complete map ({len(map_xyz)} pts, dim)")

    # live incremental map from keyframe clouds
    live_pcd = o3d.geometry.PointCloud()
    live_pcd.points = o3d.utility.Vector3dVector(np.zeros((1, 3)))
    live_pcd.colors = o3d.utility.Vector3dVector([[0.5, 0.7, 0.9]])
    vis.add_geometry(live_pcd)

    scan_pcd = o3d.geometry.PointCloud()
    scan_pcd.points = o3d.utility.Vector3dVector(np.zeros((1, 3)))
    scan_pcd.colors = o3d.utility.Vector3dVector([COLORS["scan"]])
    vis.add_geometry(scan_pcd)

    path_ls = o3d.geometry.LineSet()
    set_lineset(path_ls, np.zeros((2, 3)), COLORS["path"])
    vis.add_geometry(path_ls)

    odom_ls = o3d.geometry.LineSet()
    set_lineset(odom_ls, np.zeros((2, 3)), COLORS["odom"])
    vis.add_geometry(odom_ls)

    gt_ls = o3d.geometry.LineSet()
    set_lineset(gt_ls, np.zeros((2, 3)), COLORS["gt"])
    vis.add_geometry(gt_ls)

    pose_ls = o3d.geometry.LineSet()
    set_pose_axes(pose_ls, np.eye(4), 1.5)
    vis.add_geometry(pose_ls)

    # pose marker: single bright point (cheap)
    marker = o3d.geometry.PointCloud()
    marker.points = o3d.utility.Vector3dVector([[0.0, 0.0, 0.0]])
    marker.colors = o3d.utility.Vector3dVector([COLORS["pose"]])
    vis.add_geometry(marker)

    state = {
        "i": 0,
        "paused": False,
        "quit": False,
        "follow": bool(args.follow),
        "fit": True,
        "pause_wall": 0.0,
    }

    def on_quit(_):
        state["quit"] = True
        return False

    def on_pause(_):
        state["paused"] = not state["paused"]
        print("\npaused" if state["paused"] else "\nresume")
        return False

    def on_follow(_):
        state["follow"] = not state["follow"]
        print(f"\nfollow={'ON' if state['follow'] else 'OFF'}")
        return False

    def on_fit(_):
        state["follow"] = False
        state["fit"] = True
        print("\nauto-fit overview")
        return False

    vis.register_key_callback(ord("Q"), on_quit)
    vis.register_key_callback(256, on_quit)
    vis.register_key_callback(ord(" "), on_pause)
    vis.register_key_callback(ord("F"), on_follow)
    vis.register_key_callback(ord("A"), on_fit)

    record_dir = Path(args.record_dir).expanduser() if args.record_dir else None
    if record_dir:
        record_dir.mkdir(parents=True, exist_ok=True)

    vis.poll_events()
    vis.update_renderer()

    t0 = float(timeline_t[0])
    t_rel = timeline_t - t0
    duration = float(t_rel[-1])
    n = len(timeline_t)
    print(
        f"playback start  speed={args.speed}×  duration={duration:.1f}s  "
        f"follow={'ON' if state['follow'] else 'OFF'}  (A=overview)"
    )
    print("  dim=complete map.xyz  bright=live kf map  orange=scan  green/blue/gray=paths")

    frame_id = 0
    last_kf = -1
    last_path_ki = -1
    last_gt_j = -1
    last_odom_i = -1
    live_chunks: list[np.ndarray] = []

    def apply_view(look_p: np.ndarray, Tm: np.ndarray):
        try:
            ctr = vis.get_view_control()
        except Exception:
            return
        if state["follow"]:
            eye_offset = Tm[:3, :3] @ np.array([-12.0, 0.0, 8.0])
            ctr.set_lookat(look_p)
            ctr.set_front(-eye_offset / (np.linalg.norm(eye_offset) + 1e-9))
            ctr.set_up([0, 0, 1])
            ctr.set_zoom(0.22)
        elif state["fit"] and len(map_xyz) > 10:
            center = 0.5 * (map_xyz.min(0) + map_xyz.max(0))
            extent = float(np.max(map_xyz.max(0) - map_xyz.min(0)))
            ctr.set_lookat(center)
            ctr.set_front([0.45, -0.55, -0.7])
            ctr.set_up([0, 0, 1])
            ctr.set_zoom(max(0.12, min(0.55, 40.0 / (extent + 1e-6))))
            state["fit"] = False

    def rebuild_live(upto_ki: int):
        """Accumulate keyframe clouds 0..upto_ki into live_pcd (voxelized)."""
        nonlocal live_chunks
        while len(live_chunks) <= upto_ki:
            live_chunks.append(kfs[len(live_chunks)]["xyz"])
        pts = np.vstack(live_chunks[: upto_ki + 1])
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        if args.live_voxel > 1e-9 and pts.shape[0] > 200:
            pcd = pcd.voxel_down_sample(args.live_voxel)
        xyz = np.asarray(pcd.points)
        if xyz.shape[0] == 0:
            xyz = np.zeros((1, 3))
        live_pcd.points = o3d.utility.Vector3dVector(xyz)
        live_pcd.colors = o3d.utility.Vector3dVector(height_colors(xyz))
        vis.update_geometry(live_pcd)
        return len(xyz)

    wall0 = time.perf_counter()
    paused_at = None
    live_n = 0

    # initial fit to complete map
    apply_view(timeline_xyz[0], tum_to_T(timeline_xyz[0], timeline_q[0]))
    state["fit"] = False

    try:
        while not state["quit"]:
            if state["paused"]:
                if paused_at is None:
                    paused_at = time.perf_counter()
                if not vis.poll_events():
                    break
                vis.update_renderer()
                time.sleep(0.03)
                continue
            if paused_at is not None:
                wall0 += time.perf_counter() - paused_at
                paused_at = None

            i = state["i"]
            t_now = float(timeline_t[i])
            p = timeline_xyz[i]
            q = timeline_q[i]
            T = tum_to_T(p, q)

            ki = int(np.searchsorted(kf_t, t_now, side="right") - 1)
            ki = max(0, min(len(kfs) - 1, ki))

            if ki != last_kf:
                last_kf = ki
                live_n = rebuild_live(ki)
                sc = kfs[ki]["xyz"]
                if sc.shape[0] == 0:
                    sc = np.zeros((1, 3))
                scan_pcd.points = o3d.utility.Vector3dVector(sc)
                scan_pcd.colors = o3d.utility.Vector3dVector(
                    np.tile(COLORS["scan"], (sc.shape[0], 1))
                )
                vis.update_geometry(scan_pcd)

            if ki != last_path_ki:
                last_path_ki = ki
                set_lineset(path_ls, kf_poses[: ki + 1], COLORS["path"])
                vis.update_geometry(path_ls)

            if not args.no_odom and i != last_odom_i:
                last_odom_i = i
                set_lineset(odom_ls, timeline_xyz[: i + 1], COLORS["odom"])
                vis.update_geometry(odom_ls)

            if gt_t is not None and gt_xyz_full is not None:
                gj = int(np.searchsorted(gt_t, t_now, side="right"))
                gj = max(2, min(len(gt_t), gj))
                if gj != last_gt_j:
                    last_gt_j = gj
                    set_lineset(gt_ls, gt_xyz_full[:gj], COLORS["gt"])
                    vis.update_geometry(gt_ls)

            set_pose_axes(pose_ls, T, 1.5)
            vis.update_geometry(pose_ls)
            marker.points = o3d.utility.Vector3dVector([p])
            marker.colors = o3d.utility.Vector3dVector([COLORS["pose"]])
            vis.update_geometry(marker)

            if state["follow"] or state["fit"]:
                apply_view(p, T)

            if frame_id % 5 == 0:
                pct = 100.0 * i / max(1, n - 1)
                print(
                    f"\r  t=+{t_rel[i]:6.1f}s / {duration:.1f}s  "
                    f"odom {i+1}/{n} ({pct:5.1f}%)  kf {ki+1}/{len(kfs)}  "
                    f"live≈{live_n}  full={len(map_xyz)}   ",
                    end="",
                    flush=True,
                )

            if not vis.poll_events():
                break
            vis.update_renderer()

            if record_dir and (frame_id % max(1, args.record_stride) == 0):
                vis.capture_screen_image(
                    str(record_dir / f"frame_{frame_id:05d}.png"), do_render=False
                )
            frame_id += 1

            if i >= n - 1:
                live_n = rebuild_live(len(kfs) - 1)
                print(
                    f"\nfinished t=+{duration:.1f}s  "
                    f"full_map={len(map_xyz)}  live≈{live_n}"
                )
                if args.loop:
                    state["i"] = 0
                    last_kf = -1
                    last_path_ki = -1
                    last_gt_j = -1
                    last_odom_i = -1
                    live_chunks.clear()
                    wall0 = time.perf_counter()
                    state["fit"] = True
                    print("loop restart")
                    continue
                state["fit"] = True
                apply_view(
                    timeline_xyz[-1],
                    tum_to_T(timeline_xyz[-1], timeline_q[-1]),
                )
                for _ in range(150):
                    if state["quit"] or not vis.poll_events():
                        break
                    vis.update_renderer()
                    time.sleep(0.03)
                break

            target = float(t_rel[i + 1]) / max(args.speed, 1e-9)
            while not state["quit"] and not state["paused"]:
                remain = target - (time.perf_counter() - wall0)
                if remain <= 0:
                    break
                if not vis.poll_events():
                    state["quit"] = True
                    break
                vis.update_renderer()
                time.sleep(min(0.03, remain))
            if state["quit"]:
                break
            if not state["paused"]:
                state["i"] = i + 1
    finally:
        try:
            vis.destroy_window()
        except Exception:
            pass
    print("\ndone")


if __name__ == "__main__":
    main()
