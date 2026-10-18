/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file verify_kitti.cpp
 * @brief C++ parse real KITTI (odometry/raw/slice) and check data integrity.
 *
 * Usage:
 *   verify_kitti <kitti_dir> [--max-frames N]
 *
 * Checks: layout open, calib P2+Tr, velodyne size, image load, LiDAR→image FOV.
 */

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

#include "apps/datasets/kitti_dataset.hpp"

namespace {

int Fail(const char* msg) {
  std::cerr << "FAIL: " << msg << std::endl;
  return 1;
}

/** @brief Fraction of points with z_c>0 that land inside the image. */
double InFovRatio(const argus::datasets::KittiFrame& fr,
                  const argus::datasets::KittiCalib& calib) {
  if (fr.image.empty() || !fr.cloud || !calib.has_P2 || !calib.has_Tr) {
    return 0.0;
  }
  const int w = fr.image.cols;
  const int h = fr.image.rows;
  const Eigen::Matrix3d R = calib.R_cl();
  const Eigen::Vector3d t = calib.t_cl();
  const double fx = calib.fx();
  const double fy = calib.fy();
  const double cx = calib.cx();
  const double cy = calib.cy();

  std::size_t front = 0;
  std::size_t in_img = 0;
  constexpr std::size_t kStride = 5;
  for (std::size_t i = 0; i < fr.cloud->size(); i += kStride) {
    const auto& p = (*fr.cloud)[i];
    const Eigen::Vector3d pc = R * Eigen::Vector3d(p.x, p.y, p.z) + t;
    if (pc.z() <= 0.1) {
      continue;
    }
    ++front;
    const double u = fx * (pc.x() / pc.z()) + cx;
    const double v = fy * (pc.y() / pc.z()) + cy;
    if (u >= 0 && v >= 0 && u < w && v < h) {
      ++in_img;
    }
  }
  if (front == 0) {
    return 0.0;
  }
  return static_cast<double>(in_img) / static_cast<double>(front);
}

}  // namespace

int main(int argc, char** argv) {
  std::string root;
  int max_frames = 8;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--max-frames" && i + 1 < argc) {
      max_frames = std::stoi(argv[++i]);
    } else if (a == "--help" || a == "-h") {
      std::cout << "Usage: verify_kitti <real_kitti_dir> [--max-frames N]\n";
      return 0;
    } else if (a[0] != '-') {
      root = a;
    }
  }
  if (root.empty()) {
    // Prefer sliced real data under testdata/; else ARGUS_KITTI_ROOT.
    if (const char* env = std::getenv("ARGUS_KITTI_ROOT")) {
      root = env;
    } else {
      root = "testdata/kitti_real";
    }
  }

  argus::datasets::KittiDataset ds;
  if (!ds.Open(root)) {
    return Fail("Open real KITTI (C++ parse)");
  }
  if (ds.size() < 1) {
    return Fail("empty");
  }
  if (!ds.calib().has_P2 || !ds.calib().has_Tr) {
    return Fail("calib missing P2 or Tr");
  }
  if (ds.calib().fx() < 100.0 || ds.calib().fy() < 100.0) {
    return Fail("implausible fx/fy for KITTI camera");
  }

  const int n = std::min(max_frames, static_cast<int>(ds.size()));
  double min_fov = 1.0;
  for (int i = 0; i < n; ++i) {
    argus::datasets::KittiFrame fr;
    if (!ds.Read(static_cast<std::size_t>(i), &fr, true, false)) {
      return Fail("Read frame");
    }
    // Real Velodyne HDL-64 typically tens of thousands of points.
    if (fr.cloud->size() < 1000) {
      return Fail("velodyne too sparse (not a real scan?)");
    }
    if (fr.image.empty()) {
      return Fail("image_2/image_02 missing");
    }
    if (fr.image.cols < 600 || fr.image.rows < 200) {
      return Fail("image size not KITTI-like");
    }
    const double fov = InFovRatio(fr, ds.calib());
    min_fov = std::min(min_fov, fov);
    std::cout << "  frame=" << i << " pts=" << fr.cloud->size()
              << " img=" << fr.image.cols << "x" << fr.image.rows
              << " fov_ratio=" << fov << "\n";
  }

  // With correct Tr/P2, a non-trivial fraction of forward points hit the image.
  if (min_fov < 0.05) {
    return Fail("LiDAR→camera FOV ratio too low (calib/projection?)");
  }

  std::cout << "OK verify_kitti layout="
            << (ds.layout() == argus::datasets::KittiLayout::kOdometry
                    ? "odometry"
                    : "raw_sync")
            << " frames_checked=" << n << " min_fov=" << min_fov << "\n";
  return 0;
}
