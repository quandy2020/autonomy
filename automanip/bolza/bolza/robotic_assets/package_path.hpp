/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <cstdlib>
#include <string>

namespace bolza {
namespace robotic_assets {

/**
 * Root of robotic assets (directory containing `resources/`).
 * Priority: BOLZA_ROBOTIC_ASSETS_ROOT → env BOLZA_ROBOTIC_ASSETS →
 * in-tree `bolza/robotic_assets` (via __FILE__).
 */
inline std::string GetPath() {
#ifdef BOLZA_ROBOTIC_ASSETS_ROOT
  return std::string(BOLZA_ROBOTIC_ASSETS_ROOT);
#else
  if (const char* env = std::getenv("BOLZA_ROBOTIC_ASSETS")) {
    return std::string(env);
  }
  // …/bolza/robotic_assets/package_path.hpp → parent = package root
  const std::string here = __FILE__;
  const auto slash = here.find_last_of("/\\");
  return slash == std::string::npos ? std::string(".") : here.substr(0, slash);
#endif
}

}  // namespace robotic_assets
}  // namespace bolza
