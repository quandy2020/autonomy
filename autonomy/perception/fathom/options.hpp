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
 * @brief Validation for the single Fathom protobuf configuration model.
 */

#ifndef AUTONOMY_PERCEPTION_FATHOM_OPTIONS_HPP_
#define AUTONOMY_PERCEPTION_FATHOM_OPTIONS_HPP_

#include "autonomy/perception/fathom/proto/fathom.pb.h"

#include <string>

namespace autonomy {
namespace perception {
namespace fathom {

/**
 * @brief Validates fields required by model loading and RGB-D refinement.
 * @param options Fathom configuration loaded from protobuf text.
 * @param error Optional diagnostic output, cleared on entry.
 * @return True when the model profile is valid.
 */
bool ValidateModelOptions(const proto::FathomOptions& options,
                          std::string* error = nullptr);

/**
 * @brief Validates the complete component configuration, including topics.
 * @param options Fathom configuration loaded by autolink.
 * @param error Optional diagnostic output, cleared on entry.
 * @return True when model and transport fields are valid.
 */
bool ValidateFathomOptions(const proto::FathomOptions& options,
                           std::string* error = nullptr);

}  // namespace fathom
}  // namespace perception
}  // namespace autonomy

#endif  // AUTONOMY_PERCEPTION_FATHOM_OPTIONS_HPP_
