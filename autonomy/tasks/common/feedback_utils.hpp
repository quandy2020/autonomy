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

#pragma once

#include <functional>
#include <memory>
#include <string>

#include "autonomy/transform/buffer.hpp"

namespace autonomy {
namespace tasks {
namespace common {

struct FeedbackUtils {
    std::string robot_frame;
    std::string global_frame;
    double transform_tolerance = 0.1;
    double local_survival_timeout = 120.0;
    std::shared_ptr<autonomy::transform::Buffer> tf;
    std::string default_bt_xml_filename;
    std::function<std::string(const std::string&)> bt_xml_path_resolver;
};

}  // namespace common
}  // namespace tasks
}  // namespace autonomy
