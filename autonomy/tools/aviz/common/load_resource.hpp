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

#pragma once

#include <string>

/**
 * @brief Resource loading utilities
 * Load resource from package or path
 *
 * Provides functions to load resources (files, shaders, etc.) from various
 * locations
 */
namespace aviz {
namespace common {

/**
 * @brief Find and load a resource file
 * Searches in multiple locations:
 * 1. Relative to executable
 * 2. Install prefix (share/aviz/)
 * 3. Source directory (for development)
 *
 * @param resource_path Relative path to resource (e.g., "shaders/grid.vert")
 * @return Full path to resource file, or empty string if not found
 */
std::string findResourceFile(const std::string& resource_path);

/**
 * @brief Load resource file contents as string
 * @param resource_path Relative path to resource
 * @return File contents, or empty string if not found
 */
std::string loadResourceFile(const std::string& resource_path);

}  // namespace common
}  // namespace aviz
