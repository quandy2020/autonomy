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

#ifndef AUTONOMY_PERCEPTION_FATHOM_OPTIONS_HPP_
#define AUTONOMY_PERCEPTION_FATHOM_OPTIONS_HPP_

#include "autonomy/perception/fathom/config.hpp"
#include "autonomy/perception/fathom/proto/fathom.pb.h"

#include <string>

namespace autonomy {
namespace perception {
namespace fathom {

/** Output channels owned by the autolink component. */
struct FathomTopics {
    std::string refined_depth;
    std::string point_cloud;
};

/** Validate component options and derive the runtime/profile configuration. */
bool TranslateFathomOptions(
    const proto::FathomOptions& options,
    FathomConfig* fathom_config, FathomTopics* topics,
    std::string* error = nullptr);

}  // namespace fathom
}  // namespace perception
}  // namespace autonomy

#endif  // AUTONOMY_PERCEPTION_FATHOM_OPTIONS_HPP_
