/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
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
 * @file options.hpp
 * @brief Validation for the single Hestia protobuf configuration model.
 */

#ifndef AUTONOMY_PERCEPTION_HESTIA_OPTIONS_HPP_
#define AUTONOMY_PERCEPTION_HESTIA_OPTIONS_HPP_

#include "autonomy/perception/hestia/proto/hestia.pb.h"

#include <string>

namespace autonomy {
namespace perception {
namespace hestia {

/**
 * @brief Validates the complete Hestia runtime configuration.
 * @param options Hestia configuration loaded from protobuf text.
 * @param error Optional diagnostic output, cleared on entry.
 * @return True when mode, models, depth gates, and topics are valid.
 */
bool ValidateHestiaOptions(const proto::HestiaOptions& options,
                           std::string* error = nullptr);

/** @brief Maps Backend enum to common-network backend_id string. */
std::string BackendId(proto::Backend backend);

}  // namespace hestia
}  // namespace perception
}  // namespace autonomy

#endif  // AUTONOMY_PERCEPTION_HESTIA_OPTIONS_HPP_
