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

#include "autonomy/common/cli_options.hpp"

namespace autonomy {
namespace common {

bool FLAGS_verbose = false;
std::string FLAGS_conf;
std::string FLAGS_conf_module = "system";
std::string FLAGS_configuration_directory;
std::string FLAGS_configuration_basename;

void BindCommonOptions(CLI::App& app) {
    app.add_flag("-v,--verbose", FLAGS_verbose, "Show autonomy version info");
    app.add_option(
           "--conf", FLAGS_conf,
           "Protobuf text conf basename under autonomy/<module>/conf/ "
           "(or absolute path). Empty → process default.")
        ->capture_default_str();
    app.add_option(
           "--conf_module", FLAGS_conf_module,
           "Module name for LoadModuleConf when --conf is a basename.")
        ->capture_default_str();
    app.add_option(
           "--configuration_directory", FLAGS_configuration_directory,
           "Cartographer / legacy: directory searched for Lua configs "
           "(default: localization/conf/cartographer).")
        ->capture_default_str();
    app.add_option(
           "--configuration_basename", FLAGS_configuration_basename,
           "Cartographer / legacy: Lua config basename.")
        ->capture_default_str();
}

}  // namespace common
}  // namespace autonomy
