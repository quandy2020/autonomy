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

#include "frontend/initialization/initialization_criteria.hpp"

#include <Eigen/Core>
#include <cmath>

namespace atlas {
namespace frontend {

bool InitializationCriteria::HasValidImuWindow(
    const std::vector<sensor::ImuData>& samples,
    const InitializationOptions& options) const {
  if (options.minimum_imu_sample_count < 1 ||
      static_cast<int>(samples.size()) < options.minimum_imu_sample_count) {
    return false;
  }
  for (std::size_t index = 0; index < samples.size(); ++index) {
    const sensor::ImuData& sample = samples[index];
    if (!std::isfinite(sample.time) ||
        !sample.linear_acceleration.allFinite() ||
        !sample.angular_velocity.allFinite() ||
        (index > 0 && sample.time <= samples[index - 1].time)) {
      return false;
    }
  }
  return true;
}

bool InitializationCriteria::IsStationary(
    const std::vector<sensor::ImuData>& samples,
    const InitializationOptions& options) const {
  if (samples.empty() || !(options.maximum_acceleration_variance >= 0.0) ||
      !std::isfinite(options.maximum_acceleration_variance)) {
    return false;
  }
  Eigen::Vector3d mean_acceleration = Eigen::Vector3d::Zero();
  for (const sensor::ImuData& sample : samples) {
    if (!sample.linear_acceleration.allFinite()) {
      return false;
    }
    mean_acceleration += sample.linear_acceleration;
  }
  mean_acceleration /= static_cast<double>(samples.size());

  double acceleration_variance = 0.0;
  for (const sensor::ImuData& sample : samples) {
    acceleration_variance +=
        (sample.linear_acceleration - mean_acceleration).squaredNorm();
  }
  acceleration_variance /= static_cast<double>(samples.size());
  return acceleration_variance <= options.maximum_acceleration_variance &&
         mean_acceleration.norm() >= 1.0;
}

}  // namespace frontend
}  // namespace atlas
