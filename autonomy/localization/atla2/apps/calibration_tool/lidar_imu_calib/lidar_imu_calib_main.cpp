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

//! Lidar–IMU extrinsic / deskew calibration CLI (stub).

#include <iostream>
#include <string>

int main(int argc, char** argv) {
  std::string bag;
  std::string out = "lidar_imu_extrinsic.yaml";
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--bag" && i + 1 < argc) {
      bag = argv[++i];
    } else if (a == "--out" && i + 1 < argc) {
      out = argv[++i];
    } else if (a == "-h" || a == "--help") {
      std::cerr << "Usage: lidar_imu_calib --bag <path> [--out yaml]\n";
      return 0;
    }
  }
  if (bag.empty()) {
    std::cerr << "Usage: lidar_imu_calib --bag <path> [--out yaml]\n";
    return 1;
  }
  std::cout << "[lidar_imu_calib] stub: bag=" << bag << " out=" << out
            << " (solver TODO)\n";
  return 0;
}
