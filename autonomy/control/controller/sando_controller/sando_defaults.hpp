/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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
 * @file sando_defaults.hpp
 * @brief Fill unset SANDO options with the ground-robot defaults.
 *
 * Protobuf fields that are zero or empty are treated as unset. A field that
 * the user set explicitly, including a meaningful zero such as minimum_turn_degrees,
 * is left alone. The defaults match the ground port: differential drive,
 * horizon 8 m, 0.8 m/s, four segments, factor window 1 to 2.5 with step 0.1,
 * and a commit length of 50 samples.
 */

#pragma once

#include "autonomy/control/proto/sando_controller.pb.h"

namespace autonomy {
namespace control {
namespace controller {
namespace sando_controller {

/**
 * @brief Replace empty strings and non-positive numeric fields with defaults.
 *
 * minimum_turn_degrees of 0 is kept, because 0 disables the short-edge filter.
 * yaw_spinning_threshold defaults to 10000. factor_step defaults to 0.1.
 * default_k defaults to 50. Cluster limits are grid-cell counts, not lidar
 * return counts.
 *
 * @param options Message updated in place. Must be non-null.
 */
/**
 * @brief Fills unset ground-robot SANDO options.
 */
class SandoDefaults {
 public:
  void Apply(proto::SandoControllerOptions* options) const;
};

}  // namespace sando_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
