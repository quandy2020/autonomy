#!/usr/bin/env python3
"""Align a replay TUM trajectory to Oxford Spires base-frame ground truth."""

import argparse
import csv
from pathlib import Path

import numpy as np
import yaml
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as pyplot


def read_tum(path):
    rows = np.loadtxt(path, comments="#", ndmin=2)
    if rows.shape[1] != 8:
        raise ValueError(f"Expected eight TUM columns in {path}")
    return rows


def make_transform(translation_quaternion):
    values = np.asarray(translation_quaternion, dtype=float)
    translation, quaternion = values[:3], values[3:]
    x, y, z, w = quaternion
    rotation = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),
         2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z),
         2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w),
         1 - 2 * (x * x + y * y)],
    ])
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    return transform


def base_from_imu(calibration_directory):
    sensor = yaml.safe_load((calibration_directory / "sensor.yaml").read_text())[
        "sensor"
    ]
    camera = sensor["cameras"][0]
    camera_from_lidar = make_transform(camera["T_cam_lidar_t_xyz_q_xyzw"])
    camera_from_imu = make_transform(camera["T_cam_imu_t_xyz_q_xyzw"])
    base_from_lidar = make_transform(sensor["T_base_lidar_t_xyz_q_xyzw"])
    lidar_from_imu = np.linalg.inv(camera_from_lidar) @ camera_from_imu
    return base_from_lidar @ lidar_from_imu


def imu_trajectory_to_base(rows, base_from_imu):
    imu_from_base = np.linalg.inv(base_from_imu)
    base_rows = rows.copy()
    for index, row in enumerate(rows):
        rotation = make_transform([0, 0, 0, *row[4:8]])
        rotation[:3, 3] = row[1:4]
        world_from_base = rotation @ imu_from_base
        base_rows[index, 1:4] = world_from_base[:3, 3]
        base_rows[index, 4:8] = matrix_to_quaternion(world_from_base[:3, :3])
    return base_rows


def matrix_to_quaternion(matrix):
    trace = np.trace(matrix)
    if trace > 0:
        scale = 2 * np.sqrt(trace + 1)
        quaternion = [
            (matrix[2, 1] - matrix[1, 2]) / scale,
            (matrix[0, 2] - matrix[2, 0]) / scale,
            (matrix[1, 0] - matrix[0, 1]) / scale,
            scale / 4,
        ]
    else:
        axis = int(np.argmax(np.diag(matrix)))
        next_axis = (axis + 1) % 3
        final_axis = (axis + 2) % 3
        scale = 2 * np.sqrt(
            1 + matrix[axis, axis] - matrix[next_axis, next_axis]
            - matrix[final_axis, final_axis]
        )
        quaternion = [0.0, 0.0, 0.0, 0.0]
        quaternion[axis] = scale / 4
        quaternion[3] = (matrix[final_axis, next_axis] -
                          matrix[next_axis, final_axis]) / scale
        quaternion[next_axis] = (matrix[next_axis, axis] +
                                 matrix[axis, next_axis]) / scale
        quaternion[final_axis] = (matrix[final_axis, axis] +
                                  matrix[axis, final_axis]) / scale
    return np.asarray(quaternion)


def match_trajectories(estimated, ground_truth, maximum_time_difference):
    matched_estimated, matched_ground_truth = [], []
    timestamps = ground_truth[:, 0]
    for row in estimated:
        right = int(np.searchsorted(timestamps, row[0]))
        options = [index for index in (right - 1, right)
                   if 0 <= index < len(timestamps)]
        nearest = min(options, key=lambda index: abs(timestamps[index] - row[0]))
        if abs(timestamps[nearest] - row[0]) <= maximum_time_difference:
            matched_estimated.append(row)
            matched_ground_truth.append(ground_truth[nearest])
    return np.asarray(matched_estimated), np.asarray(matched_ground_truth)


def align_positions(estimated_positions, ground_truth_positions):
    estimated_mean = estimated_positions.mean(axis=0)
    ground_truth_mean = ground_truth_positions.mean(axis=0)
    centered_estimated = estimated_positions - estimated_mean
    centered_ground_truth = ground_truth_positions - ground_truth_mean
    covariance = np.einsum(
        "ni,nj->ij", centered_estimated, centered_ground_truth
    )
    left, _, right = np.linalg.svd(covariance)
    correction = np.eye(3)
    correction[2, 2] = np.linalg.det(right.T @ left.T)
    rotation = right.T @ correction @ left.T
    translation = ground_truth_mean - rotation @ estimated_mean
    return rotation, translation


def compute_translation_metrics(position_errors):
    ate_mean = float(np.mean(position_errors))
    translation_rmse = float(np.sqrt(np.mean(position_errors ** 2)))
    return ate_mean, translation_rmse


def save_trajectory_plot(estimated, ground_truth, position_errors, ate_mean,
                        translation_rmse, output_path):
    figure, axes = pyplot.subplots(1, 3, figsize=(18, 6))
    axes[0].plot(ground_truth[:, 1], ground_truth[:, 2], label="Ground truth")
    axes[0].plot(estimated[:, 1], estimated[:, 2], label="Atlas estimate")
    axes[0].set_title("Aligned trajectory")
    axes[0].set_xlabel("X (m)")
    axes[0].set_ylabel("Y (m)")
    axes[0].axis("equal")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()

    elapsed_time = estimated[:, 0] - estimated[0, 0]
    axes[1].plot(elapsed_time, position_errors, color="tab:red")
    axes[1].axhline(ate_mean, color="tab:orange", linestyle="--",
                    label=f"ATE mean: {ate_mean:.3f} m")
    axes[1].axhline(translation_rmse, color="tab:blue", linestyle=":",
                    label=f"RMSE: {translation_rmse:.3f} m")
    axes[1].set_title("Position error")
    axes[1].set_xlabel("Elapsed time (s)")
    axes[1].set_ylabel("Error (m)")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()

    axes[2].axis("off")
    axes[2].set_title("Trajectory metrics")
    axes[2].text(
        0.03, 0.75,
        f"ATE mean translation error\n{ate_mean:.6f} m\n\n"
        f"Translation RMSE\n{translation_rmse:.6f} m\n\n"
        f"Matched poses\n{len(estimated)}",
        fontsize=15, va="top")
    figure.tight_layout()
    figure.savefig(output_path, dpi=180, bbox_inches="tight")
    pyplot.close(figure)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--estimated", type=Path, required=True)
    parser.add_argument("--ground-truth", type=Path, required=True)
    parser.add_argument("--calibration-directory", type=Path, required=True)
    parser.add_argument("--output-directory", type=Path, required=True)
    parser.add_argument("--maximum-time-difference", type=float, default=0.05)
    arguments = parser.parse_args()

    estimated = imu_trajectory_to_base(
        read_tum(arguments.estimated), base_from_imu(arguments.calibration_directory)
    )
    ground_truth = read_tum(arguments.ground_truth)
    estimated, ground_truth = match_trajectories(
        estimated, ground_truth, arguments.maximum_time_difference
    )
    if len(estimated) < 3:
        raise RuntimeError(f"Only {len(estimated)} timestamp matches")

    rotation, translation = align_positions(estimated[:, 1:4], ground_truth[:, 1:4])
    aligned_positions = (
        np.einsum("ij,nj->ni", rotation, estimated[:, 1:4]) + translation
    )
    position_errors = np.linalg.norm(aligned_positions - ground_truth[:, 1:4], axis=1)
    ate_mean, translation_rmse = compute_translation_metrics(position_errors)
    output_directory = arguments.output_directory
    output_directory.mkdir(parents=True, exist_ok=True)
    with (output_directory / "trajectory_comparison.csv").open("w") as output_file:
        writer = csv.writer(output_file)
        writer.writerow(["timestamp", "estimated_x", "estimated_y", "estimated_z",
                         "ground_truth_x", "ground_truth_y", "ground_truth_z",
                         "position_error_meters"])
        for estimate, truth, error in zip(estimated, ground_truth, position_errors):
            writer.writerow([estimate[0], *estimate[1:4], *truth[1:4], error])
    aligned_tum = estimated.copy()
    aligned_tum[:, 1:4] = aligned_positions
    np.savetxt(output_directory / "estimated_base_aligned.tum", aligned_tum,
               fmt="%.12f")
    np.savetxt(output_directory / "ground_truth_matched.tum", ground_truth,
               fmt="%.12f")
    save_trajectory_plot(aligned_tum, ground_truth, position_errors, ate_mean,
                         translation_rmse,
                         output_directory / "ate_rmse_plot.png")
    print(f"matches: {len(estimated)}/{len(read_tum(arguments.estimated))}")
    print(f"maximum time gap: {np.max(np.abs(estimated[:, 0] - ground_truth[:, 0])):.6f} s")
    print(f"ATE mean translation error: {ate_mean:.6f} m")
    print(f"translation RMSE: {translation_rmse:.6f} m")
    print(f"translation median/P95/max: {np.median(position_errors):.6f} "
          f"{np.percentile(position_errors, 95):.6f} "
          f"{position_errors.max():.6f} m")


if __name__ == "__main__":
    main()
