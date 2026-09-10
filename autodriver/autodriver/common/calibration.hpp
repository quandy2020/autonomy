/*
 * Copyright 2026 Autodriver contributors
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
 * @file
 * @brief Load static sensor extrinsics from YAML (ROS-like transform layout).
 */

#ifndef AUTODRIVER_COMMON_CALIBRATION_HPP_
#define AUTODRIVER_COMMON_CALIBRATION_HPP_

#include <string>

#include <Eigen/Geometry>

namespace autodriver {
namespace common {

/**
 * @brief Static rigid transform from child frame into parent frame.
 *
 * Convention: @c transform maps points in @c child_frame into @c parent_frame
 * (parent ← child), matching typical lidar/camera extrinsic YAML used with
 * Velodyne-style params files.
 */
struct Extrinsic {
  /** Parent / reference frame id (e.g. "base_link", "novatel"). */
  std::string parent_frame;
  /** Child / sensor frame id (e.g. "velodyne", "camera_link"). */
  std::string child_frame;
  /** SE(3) transform parent ← child; identity when unset. */
  Eigen::Affine3d transform = Eigen::Affine3d::Identity();
};

/**
 * @brief Parse an extrinsic YAML file into @p out.
 *
 * Accepted layout (fields may be nested under @c header / @c transform):
 * - @c header.frame_id or top-level @c frame_id → @c Extrinsic::parent_frame
 * - @c child_frame_id → @c Extrinsic::child_frame
 * - @c transform.translation.{x,y,z} and @c transform.rotation.{x,y,z,w}
 *   (quaternion); if @c transform is absent, translation/rotation may sit at
 *   the root.
 *
 * @param path Absolute or relative path to the YAML file.
 * @param out Non-null destination; filled only on success.
 * @param error Optional human-readable failure reason (YAML parse / null out).
 * @return true on success; false leaves @p out unchanged (except when @p out
 *         is null).
 */
bool LoadExtrinsicYaml(const std::string& path, Extrinsic* out,
                       std::string* error = nullptr);

}  // namespace common
}  // namespace autodriver

#endif  // AUTODRIVER_COMMON_CALIBRATION_HPP_
