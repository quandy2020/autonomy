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

namespace autonomy {
namespace tasks {
namespace utils {

/** Map BT/XML aliases (e.g. GridBased) to configured planner plugin ids. */
inline std::string ResolvePlannerId(const std::string& planner_id,
                                    const std::string& default_planner_id) {
    if (planner_id == "GridBased" || planner_id == "grid_based") {
        return default_planner_id.empty() ? "navfn_planner" : default_planner_id;
    }
    if (planner_id.empty()) {
        return default_planner_id.empty() ? "navfn_planner" : default_planner_id;
    }
    return planner_id;
}

}  // namespace utils
}  // namespace tasks
}  // namespace autonomy
