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
 * @file slice_kitti.cpp
 * @brief Copy a short real KITTI odometry/raw slice into testdata/ (no synthesis).
 *
 * Usage:
 *   slice_kitti --src <KITTI_seq_or_raw_sync> --dst testdata/kitti_real \
 *               [--frames N] [--start I]
 *
 * Example (odometry seq 00):
 *   slice_kitti --src /data/kitti/sequences/00 --dst testdata/kitti_real --frames 3
 */

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include <opencv2/imgcodecs.hpp>

#include "apps/datasets/kitti_dataset.hpp"

namespace fs = std::filesystem;

namespace {

bool CopyFile(const fs::path& from, const fs::path& to) {
  std::error_code ec;
  fs::create_directories(to.parent_path(), ec);
  fs::copy_file(from, to, fs::copy_options::overwrite_existing, ec);
  return !ec;
}

}  // namespace

int main(int argc, char** argv) {
  std::string src;
  std::string dst = "testdata/kitti_real";
  int frames = 3;
  int start = 0;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto need = [&](const char* n) -> std::string {
      if (i + 1 >= argc) {
        std::cerr << "missing " << n << "\n";
        std::exit(2);
      }
      return argv[++i];
    };
    if (a == "--src") {
      src = need("--src");
    } else if (a == "--dst") {
      dst = need("--dst");
    } else if (a == "--frames") {
      frames = std::stoi(need("--frames"));
    } else if (a == "--start") {
      start = std::stoi(need("--start"));
    } else if (a == "--help" || a == "-h") {
      std::cout << "Usage: slice_kitti --src <real_kitti> --dst <out> "
                   "[--frames N] [--start I]\n"
                << "Copies real frames only (velodyne + image + calib).\n";
      return 0;
    }
  }
  if (src.empty()) {
    std::cerr << "need --src <KITTI odometry seq or raw sync dir>\n";
    return 2;
  }

  argus::datasets::KittiDataset ds;
  if (!ds.Open(src)) {
    return 1;
  }
  if (start < 0 || start >= static_cast<int>(ds.size())) {
    std::cerr << "bad --start\n";
    return 1;
  }
  const int n = std::min(frames, static_cast<int>(ds.size()) - start);

  fs::remove_all(dst);
  fs::create_directories(fs::path(dst) / "velodyne");
  fs::create_directories(fs::path(dst) / "image_2");

  // Always write odometry-style layout for the slice (easier offline tools).
  std::ofstream times((fs::path(dst) / "times.txt").string());
  for (int i = 0; i < n; ++i) {
    argus::datasets::KittiFrame fr;
    if (!ds.Read(static_cast<std::size_t>(start + i), &fr, true, false)) {
      std::cerr << "Read failed at " << (start + i) << "\n";
      return 1;
    }
    char stem[32];
    std::snprintf(stem, sizeof(stem), "%06d", i);
    times << fr.timestamp << '\n';

    // Re-write velodyne bin from parsed cloud (same float layout).
    {
      std::ofstream bin(
          (fs::path(dst) / "velodyne" / (std::string(stem) + ".bin")).string(),
          std::ios::binary);
      for (std::size_t k = 0; k < fr.cloud->size(); ++k) {
        const auto& p = (*fr.cloud)[k];
        const float xyz[4] = {p.x, p.y, p.z, p.intensity};
        bin.write(reinterpret_cast<const char*>(xyz), sizeof(xyz));
      }
    }
    if (!fr.image.empty()) {
      const fs::path img =
          fs::path(dst) / "image_2" / (std::string(stem) + ".png");
      if (!cv::imwrite(img.string(), fr.image)) {
        std::cerr << "imwrite failed " << img << "\n";
        return 1;
      }
    }
  }

  // Calib: copy odometry calib.txt or synthesize from parsed KittiCalib.
  if (ds.layout() == argus::datasets::KittiLayout::kOdometry) {
    CopyFile(fs::path(src) / "calib.txt", fs::path(dst) / "calib.txt");
  } else {
    const auto& c = ds.calib();
    std::ofstream cal((fs::path(dst) / "calib.txt").string());
    cal << "P2:";
    for (int r = 0; r < 3; ++r) {
      for (int col = 0; col < 4; ++col) {
        cal << ' ' << c.P2(r, col);
      }
    }
    cal << "\nTr:";
    for (int r = 0; r < 3; ++r) {
      for (int col = 0; col < 4; ++col) {
        cal << ' ' << c.Tr(r, col);
      }
    }
    cal << '\n';
  }

  std::ofstream((fs::path(dst) / "README.md").string())
      << "# Real KITTI slice\n\n"
         "Copied from `"
      << src << "` frames [" << start << ", " << (start + n) << ").\n"
      << "Source layout: "
      << (ds.layout() == argus::datasets::KittiLayout::kOdometry ? "odometry"
                                                               : "raw_sync")
      << "\n\nVerify: `verify_kitti " << dst << "`\n";

  std::cout << "[slice_kitti] copied " << n << " real frames → " << dst << "\n";
  return 0;
}
