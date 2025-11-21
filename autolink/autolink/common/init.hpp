/**
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

#include "autolink/common/log.hpp"
#include "autolink/common/state.hpp"

namespace autolink {

/**
 * @brief Initialize the autolink framework
 * @param binary_name The name of the binary
 * @param dag_info The DAG information
 * @return True if the initialization is successful, false otherwise
 */
bool Init(const char* binary_name, const std::string& dag_info = "");

/**
 * @brief Clear the autolink framework
 */
void Clear();

/**
 * @brief On shutdown signal
 * @param sig The signal
 */
void OnShutdown(int sig);

}  // namespace autolink