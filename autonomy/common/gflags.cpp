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

#include "autonomy/common/gflags.hpp"

namespace autonomy {
namespace common {

DEFINE_bool(verbose, false, "Show autonomy version info");

DEFINE_string(conf, "",
              "Protobuf text conf basename under autonomy/<module>/conf/ "
              "(or absolute path). Empty → process default "
              "(system: autonomy.pb.txt, bridge: bridge.pb.txt, …). "
              "Localization still uses configuration_directory for Cartographer.");
DEFINE_string(conf_module, "system",
              "Module name for LoadModuleConf when --conf is a basename.");

DEFINE_string(configuration_directory, "",
              "Cartographer / legacy: directory searched for Lua configs "
              "(default: localization/conf/cartographer).");
DEFINE_string(configuration_basename, "",
              "Cartographer / legacy: Lua config basename.");

}  // namespace common
}  // namespace autonomy
