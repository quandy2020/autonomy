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

#include <string>

#include "autonomy/tasks/common/feedback_utils.hpp"

namespace autonomy {
namespace tasks {
namespace navigator {
namespace utils {

inline std::string ResolveBehaviorTreeFile(
    const std::string& bt_file,
    const common::FeedbackUtils& feedback) {
    if (bt_file.empty()) {
        return bt_file;
    }
    if (bt_file.find('/') != std::string::npos) {
        return bt_file;
    }
    if (feedback.bt_xml_path_resolver) {
        return feedback.bt_xml_path_resolver(bt_file);
    }
    return bt_file;
}

}  // namespace utils
}  // namespace navigator
}  // namespace tasks
}  // namespace autonomy
