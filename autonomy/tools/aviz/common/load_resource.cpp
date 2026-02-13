/******************************************************************************
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "autonomy/tools/aviz/common/load_resource.hpp"

#include <fstream>
#include <iostream>
#include <sstream>

#include "autolink/common/environment.hpp"
#include "autolink/common/file.hpp"

namespace aviz {
namespace common {

std::string findResourceFile(const std::string& resource_path) {
    // Try multiple possible locations
    std::vector<std::string> search_paths;

    // 1. Relative to current working directory
    search_paths.push_back(resource_path);

    // 2. Try relative to source directory (for development)
    // This assumes the build directory structure
    search_paths.push_back("src/autonomy/autonomy/tools/aviz/" + resource_path);

    // 3. Try standard install prefixes
    search_paths.push_back("/usr/local/share/aviz/" + resource_path);
    search_paths.push_back("/usr/share/aviz/" + resource_path);

    // 4. Try AUTOLINK_PATH if set
    std::string autolink_path = autolink::common::GetEnv("AUTOLINK_PATH");
    if (!autolink_path.empty()) {
        search_paths.push_back(autolink::common::GetAbsolutePath(autolink_path, "share/aviz/" + resource_path));
    }

    // Try each path
    for (const auto& path : search_paths) {
        std::ifstream file(path);
        if (file.good()) {
            file.close();
            return path;
        }
    }

    return "";
}

std::string loadResourceFile(const std::string& resource_path) {
    std::string full_path = findResourceFile(resource_path);
    if (full_path.empty()) {
        std::cerr << "aviz_common: Resource not found: " << resource_path << std::endl;
        return "";
    }

    std::ifstream file(full_path);
    if (!file.is_open()) {
        std::cerr << "aviz_common: Failed to open resource: " << full_path << std::endl;
        return "";
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

}  // namespace common
}  // namespace aviz
