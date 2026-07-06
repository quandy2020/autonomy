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

#pragma once

#include <string>

namespace autonomy {
namespace localization {
namespace cartographer {
namespace node {

struct CartographerNodeFlags {
    std::string configuration_directory;
    std::string configuration_basename;
    std::string load_state_filename;
    bool load_frozen_state = true;
    bool start_trajectory_with_default_topics = true;
    std::string save_state_filename;
};

int RunCartographerNode(const CartographerNodeFlags& flags);

}  // namespace node
}  // namespace cartographer
}  // namespace localization
}  // namespace autonomy
