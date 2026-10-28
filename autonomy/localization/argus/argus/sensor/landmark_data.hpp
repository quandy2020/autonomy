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

#ifndef ARGUS_SENSOR_LANDMARK_DATA_HPP_
#define ARGUS_SENSOR_LANDMARK_DATA_HPP_

#include <string>
#include <vector>

#include "common/transform/rigid_transform.hpp"

namespace argus {
namespace sensor {

struct LandmarkObservation {
  std::string id;
  transform::Rigid3d landmark_to_tracking_transform =
      transform::Rigid3d::Identity();
  double translation_weight = 0.0;
  double rotation_weight = 0.0;
};

struct LandmarkData {
  double time = 0.0;
  std::vector<LandmarkObservation> landmark_observations;
};

}  // namespace sensor
}  // namespace argus

#endif  // ARGUS_SENSOR_LANDMARK_DATA_HPP_
