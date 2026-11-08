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

#ifndef ATLAS_APP_FILE_REPLAY_SOURCE_HPP_
#define ATLAS_APP_FILE_REPLAY_SOURCE_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include "sensor/imu_data.hpp"
#include "sensor/point_types.hpp"

namespace atlas {
namespace app {

/** Reads text datasets whose scan point times are relative seconds. */
class FileReplaySource {
 public:
  bool Load(const std::string& directory,
            double point_time_scale_to_seconds = 1.0);
  bool LoadScan(std::size_t index, sensor::PointCloudXYZI* cloud,
                std::string* error) const;

  const std::string& directory() const { return directory_; }
  const std::vector<sensor::ImuData>& imu() const { return imu_; }
  std::size_t scan_count() const { return scans_.size(); }
  double scan_time(std::size_t index) const { return scans_[index].time; }

 private:
  struct ScanFile {
    double time = 0.0;
    std::string path;
  };

  std::string directory_;
  double point_time_scale_to_seconds_ = 1.0;
  std::vector<sensor::ImuData> imu_;
  std::vector<ScanFile> scans_;
};

}  // namespace app
}  // namespace atlas

#endif  // ATLAS_APP_FILE_REPLAY_SOURCE_HPP_
