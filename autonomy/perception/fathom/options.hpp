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
 * @brief Translation from protobuf component options to runtime settings.
 */

#ifndef AUTONOMY_PERCEPTION_FATHOM_OPTIONS_HPP_
#define AUTONOMY_PERCEPTION_FATHOM_OPTIONS_HPP_

#include "autonomy/perception/fathom/config.hpp"
#include "autonomy/perception/fathom/proto/fathom.pb.h"

#include <string>

namespace autonomy {
namespace perception {
namespace fathom {

/** @brief Validated output channels owned by the Fathom component. */
struct FathomTopics {
    /** Topic for the refined `32FC1` depth image. */
    std::string refined_depth;
    /** Topic for the organized XYZ point cloud. */
    std::string point_cloud;
};

/**
 * @brief Validates protobuf options and derives internal runtime settings.
 * @param options Component configuration loaded by autolink.
 * @param fathom_config Output model and preprocessing profile.
 * @param topics Output publication topics.
 * @param error Optional diagnostic output, cleared on entry.
 * @return True when the complete component configuration is valid.
 */
bool TranslateFathomOptions(const proto::FathomOptions& options,
                            FathomConfig* fathom_config, FathomTopics* topics,
                            std::string* error = nullptr);

}  // namespace fathom
}  // namespace perception
}  // namespace autonomy

#endif  // AUTONOMY_PERCEPTION_FATHOM_OPTIONS_HPP_
