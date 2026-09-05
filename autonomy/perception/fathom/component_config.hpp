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

#ifndef AUTONOMY_PERCEPTION_FATHOM_COMPONENT_CONFIG_HPP_
#define AUTONOMY_PERCEPTION_FATHOM_COMPONENT_CONFIG_HPP_

#include "autonomy/perception/fathom/config.hpp"
#include "autonomy/perception/fathom/proto/fathom_component_config.pb.h"

#include <string>

namespace autonomy {
namespace perception {
namespace fathom {

struct FathomComponentTopics {
    std::string refined_depth;
    std::string point_cloud;
};

bool TranslateFathomComponentConfig(
    const proto::FathomComponentConfig& component_config,
    FathomConfig* fathom_config, FathomComponentTopics* topics,
    std::string* error = nullptr);

}  // namespace fathom
}  // namespace perception
}  // namespace autonomy

#endif  // AUTONOMY_PERCEPTION_FATHOM_COMPONENT_CONFIG_HPP_
