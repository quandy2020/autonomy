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

#ifndef ATLAS_FRONTEND_INITIALIZATION_INITIALIZATION_OPTIONS_HPP_
#define ATLAS_FRONTEND_INITIALIZATION_INITIALIZATION_OPTIONS_HPP_

namespace atlas {
namespace frontend {

struct InitializationOptions {
  int minimum_imu_sample_count = 20;
  int minimum_motion_frame_count = 3;
  double gravity_magnitude = 9.81;
  double gravity_tolerance = 0.5;
  double maximum_acceleration_variance = 0.05;
};

}  // namespace frontend
}  // namespace atlas

#endif  // ATLAS_FRONTEND_INITIALIZATION_INITIALIZATION_OPTIONS_HPP_
