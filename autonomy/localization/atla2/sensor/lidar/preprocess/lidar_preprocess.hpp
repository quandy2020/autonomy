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

#pragma once

#include "autonomy/localization/atla2/sensor/interface/types.hpp"

namespace autonomy::localization::atla2 {

struct LidarPreprocessOptions {
  float voxel_size = 0.2f;
  float min_range = 0.5f;
  float max_range = 80.0f;
  //! Axis-aligned ROI in lidar frame (disable with empty).
  bool use_roi = false;
  float roi_xmin = -50.f, roi_xmax = 50.f;
  float roi_ymin = -50.f, roi_ymax = 50.f;
  float roi_zmin = -5.f, roi_zmax = 5.f;
};

//! Range filter + optional ROI + voxel downsample.
class LidarPreprocessor {
 public:
  explicit LidarPreprocessor(LidarPreprocessOptions opt = {});

  void SetOptions(const LidarPreprocessOptions& opt) { opt_ = opt; }
  LidarScan Process(const LidarScan& in) const;

 private:
  LidarPreprocessOptions opt_;
};

}  // namespace autonomy::localization::atla2
