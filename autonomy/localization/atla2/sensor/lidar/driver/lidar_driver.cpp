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

#include "autonomy/localization/atla2/sensor/lidar/driver/lidar_driver.hpp"

namespace autonomy::localization::atla2 {

std::unique_ptr<LidarDriverBase> CreateLidarDriver(LidarModel model) {
  switch (model) {
    case LidarModel::kLivox:
      return std::make_unique<LivoxDriver>();
    case LidarModel::kOuster:
      return std::make_unique<OusterDriver>();
    case LidarModel::kVelodyne:
      return std::make_unique<VelodyneDriver>();
    case LidarModel::kGeneric:
    default:
      return std::make_unique<GenericLidarDriver>();
  }
}

}  // namespace autonomy::localization::atla2
