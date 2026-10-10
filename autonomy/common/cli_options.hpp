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

#include <cstdlib>
#include <string>

#include <CLI/CLI.hpp>

namespace autonomy {
namespace common {

// Process-wide CLI values (formerly gflags). Bound by BindCommonOptions.
extern bool FLAGS_verbose;
extern std::string FLAGS_conf;
extern std::string FLAGS_conf_module;
extern std::string FLAGS_configuration_directory;
extern std::string FLAGS_configuration_basename;

/** Add shared autonomy process flags to ``app`` (``--conf``, …). */
void BindCommonOptions(CLI::App& app);

/** Parse argv; on error print help and ``std::_Exit``. */
inline void ParseOrExit(CLI::App& app, int argc, char** argv) {
    try {
        app.parse(argc, argv);
    } catch (const CLI::ParseError& e) {
        std::_Exit(app.exit(e));
    }
}

}  // namespace common
}  // namespace autonomy
