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
 * @file lidar_vendor.hpp
 * @brief Multi-vendor LiDAR preprocess for Voxel-SLAM (Argus PointCloud only).
 *
 * Host already decoded PointCloud2 → PointXYZINormal; this applies blind zone,
 * stride filter, and relative-time in `curvature` when missing.
 */

#ifndef ARGUS_VXSLAM_LIDAR_VENDOR_HPP_
#define ARGUS_VXSLAM_LIDAR_VENDOR_HPP_

#include "sensor/point_types.hpp"

namespace argus {
namespace vxslam {

enum class LidarVendor {
  kGeneric = 0,
  kLivox = 1,
  kVelodyne = 2,
  kOuster = 3,
  kHesai = 4,
  kRobosense = 5,
};

struct LidarVendorOptions {
  LidarVendor vendor = LidarVendor::kGeneric;
  double blind = 0.5;
  int point_filter_num = 3;
  /** @brief When true, synthesize relative time 0..scan_period if curvature≈0.
   */
  bool synthesize_time = true;
  double scan_period = 0.1;
};

/**
 * @brief Filter / time-stamp a decoded cloud for Voxel-SLAM.
 * @param in Input cloud (sensor frame).
 * @param out Filtered cloud; curvature holds relative time (s).
 * @param opt Vendor options.
 */
void ProcessLidarVendor(const sensor::PointCloudXYZI& in,
                        sensor::PointCloudXYZI* out,
                        const LidarVendorOptions& opt);

}  // namespace vxslam
}  // namespace argus

#endif  // ARGUS_VXSLAM_LIDAR_VENDOR_HPP_
