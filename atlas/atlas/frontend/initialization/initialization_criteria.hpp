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

#ifndef ATLAS_FRONTEND_INITIALIZATION_INITIALIZATION_CRITERIA_HPP_
#define ATLAS_FRONTEND_INITIALIZATION_INITIALIZATION_CRITERIA_HPP_

#include <vector>

#include "frontend/initialization/initialization_options.hpp"
#include "sensor/imu_data.hpp"

namespace atlas {
namespace frontend {

class InitializationCriteria {
 public:
  bool HasValidImuWindow(const std::vector<sensor::ImuData>& samples,
                         const InitializationOptions& options) const;

  bool IsStationary(const std::vector<sensor::ImuData>& samples,
                    const InitializationOptions& options) const;
};

}  // namespace frontend
}  // namespace atlas

#endif  // ATLAS_FRONTEND_INITIALIZATION_INITIALIZATION_CRITERIA_HPP_
