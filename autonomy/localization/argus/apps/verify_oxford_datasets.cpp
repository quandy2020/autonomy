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
 * @file verify_oxford_datasets.cpp
 * @brief Smoke-check Oxford Spires / Newer College raw readers.
 *
 * Usage:
 *   verify_oxford_datasets --spires <seq_or_raw_dir> [--max-frames N]
 *   verify_oxford_datasets --ncd <raw_dir> [--max-frames N] [--camera cam0]
 */

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <string>

#include "apps/datasets/newer_college_dataset.hpp"
#include "apps/datasets/oxford_spires_dataset.hpp"

namespace {

int Fail(const char* msg) {
  std::cerr << "FAIL: " << msg << std::endl;
  return 1;
}

int VerifySpires(const std::string& root, int max_frames,
                 const std::string& camera) {
  argus::datasets::OxfordSpiresOptions opt;
  opt.camera = camera.empty() ? "cam0" : camera;
  argus::datasets::OxfordSpiresDataset ds;
  if (!ds.Open(root, opt)) {
    return Fail("OxfordSpires Open");
  }
  if (ds.size() < 1) {
    return Fail("empty lidar");
  }
  const int n = std::min(max_frames, static_cast<int>(ds.size()));
  for (int i = 0; i < n; ++i) {
    argus::datasets::OxfordSpiresFrame fr;
    if (!ds.Read(static_cast<std::size_t>(i), &fr, true)) {
      return Fail("Read frame");
    }
    if (!fr.cloud || fr.cloud->size() < 100) {
      return Fail("sparse PCD");
    }
    std::cout << "  spires frame=" << i << " pts=" << fr.cloud->size()
              << " img="
              << (fr.image.empty()
                      ? "none"
                      : std::to_string(fr.image.cols) + "x" +
                            std::to_string(fr.image.rows))
              << " dt=" << (fr.image_timestamp - fr.timestamp) << "\n";
  }
  std::cout << "OK verify_oxford_datasets spires frames=" << n
            << " K=" << ds.calib().has_intrinsics
            << " T_cl=" << ds.calib().has_extrinsics << "\n";
  return 0;
}

int VerifyNcd(const std::string& root, int max_frames,
              const std::string& camera) {
  argus::datasets::NewerCollegeOptions opt;
  opt.camera = camera;
  argus::datasets::NewerCollegeDataset ds;
  if (!ds.Open(root, opt)) {
    return Fail("NewerCollege Open");
  }
  if (ds.size() < 1) {
    return Fail("empty lidar");
  }
  const int n = std::min(max_frames, static_cast<int>(ds.size()));
  for (int i = 0; i < n; ++i) {
    argus::datasets::NewerCollegeFrame fr;
    if (!ds.Read(static_cast<std::size_t>(i), &fr, true)) {
      return Fail("Read frame");
    }
    if (!fr.cloud || fr.cloud->size() < 100) {
      return Fail("sparse PCD");
    }
    std::cout << "  ncd frame=" << i << " pts=" << fr.cloud->size()
              << " img="
              << (fr.image.empty()
                      ? "none"
                      : std::to_string(fr.image.cols) + "x" +
                            std::to_string(fr.image.rows))
              << " cam=" << fr.camera_id << "\n";
  }
  std::cout << "OK verify_oxford_datasets ncd frames=" << n
            << " layout="
            << (ds.layout() == argus::datasets::NewerCollegeLayout::kMultiCam
                    ? "multi_cam"
                    : "stereo")
            << "\n";
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  std::string mode;
  std::string root;
  std::string camera;
  int max_frames = 3;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if ((a == "--spires" || a == "--ncd") && i + 1 < argc) {
      mode = a.substr(2);
      root = argv[++i];
    } else if (a == "--max-frames" && i + 1 < argc) {
      max_frames = std::stoi(argv[++i]);
    } else if (a == "--camera" && i + 1 < argc) {
      camera = argv[++i];
    } else if (a == "--help" || a == "-h") {
      std::cout << "Usage:\n"
                   "  verify_oxford_datasets --spires <seq|raw> [--camera cam0]\n"
                   "  verify_oxford_datasets --ncd <raw_dir> [--camera cam0]\n"
                   "  [--max-frames N]\n";
      return 0;
    }
  }
  if (root.empty()) {
    if (const char* env = std::getenv("ARGUS_SPIRES_ROOT")) {
      mode = "spires";
      root = env;
    } else if (const char* env = std::getenv("ARGUS_NCD_ROOT")) {
      mode = "ncd";
      root = env;
    } else {
      std::cerr << "Need --spires/--ncd path or ARGUS_SPIRES_ROOT / "
                   "ARGUS_NCD_ROOT\n";
      return 2;
    }
  }
  if (mode == "spires") {
    return VerifySpires(root, max_frames, camera);
  }
  if (mode == "ncd") {
    return VerifyNcd(root, max_frames, camera);
  }
  return Fail("unknown mode");
}
