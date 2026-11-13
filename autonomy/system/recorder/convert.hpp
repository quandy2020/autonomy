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

#pragma once

/**
 * @file convert.hpp
 * @brief MCAP → LeRobot v2.1 layout (JSONL frames; parquet optional downstream).
 */

#include <string>

namespace autonomy {
namespace system {
namespace recorder {

/**
 * @brief Convert an episode MCAP into a LeRobot v2.1 directory (JSONL frames).
 *
 * Currently emits odom + cmd_vel JSONL. Image/BEV tensors are not converted.
 *
 * @param [in] mcap_path: Source MCAP.
 * @param [in] out_dir: Output directory (created if missing).
 * @param [in] fps: Nominal frame rate for the JSONL index.
 * @return True on success.
 */
bool ConvertMcapToLeRobot(const std::string& mcap_path,
                          const std::string& out_dir, double fps);

}  // namespace recorder
}  // namespace system
}  // namespace autonomy
