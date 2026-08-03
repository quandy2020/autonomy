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

#include "autonomy/system/options.hpp"

#include "autonomy/common/configuration_file_resolver.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/common/lua_parameter_dictionary.hpp"
#include "autonomy/control/control_options.hpp"
#include "autonomy/map/map_options.hpp"
#include "autonomy/planning/planner_options.hpp"
#include "autonomy/navigator/options.hpp"
#include "autonomy/transform/common/transform_interface.hpp"

namespace autonomy {
namespace system {
namespace {

proto::AutonomyOptions LoadOptions(
    ::autonomy::common::LuaParameterDictionary* const parameter_dictionary) {
    proto::AutonomyOptions options;

    if (parameter_dictionary->HasKey("map")) {
        *options.mutable_map_options() =
            map::LoadOptions(parameter_dictionary->GetDictionary("map").get());
    }
    if (parameter_dictionary->HasKey("controller")) {
        *options.mutable_controller_options() = control::LoadOptions(
            parameter_dictionary->GetDictionary("controller").get());
    }
    if (parameter_dictionary->HasKey("planning")) {
        *options.mutable_planner_options() = planning::LoadOptions(
            parameter_dictionary->GetDictionary("planning").get());
    }
    if (parameter_dictionary->HasKey("exploration")) {
        // Exploration module is temporarily excluded from the library build
        // during the automsgs field-access migration.
        AWARN << "system::LoadOptions: skipping exploration options "
                 "(module temporarily disabled)";
    }
    if (parameter_dictionary->HasKey("navigator")) {
        *options.mutable_navigator_options() =
            navigator::LoadOptions(parameter_dictionary);
    }
    if (parameter_dictionary->HasKey("transform")) {
        *options.mutable_transform_options() =
            transform::common::LoadOptions(
                parameter_dictionary->GetDictionary("transform").get());
    }
    return options;
}
}  // namespace

proto::AutonomyOptions CreateOptions(
    const std::string& configuration_directory,
    const std::string& configuration_basename) {
    auto file_resolver =
        std::make_unique<::autonomy::common::ConfigurationFileResolver>(
            std::vector<std::string>{configuration_directory});
    const std::string code =
        file_resolver->GetFileContentOrDie(configuration_basename);
    ::autonomy::common::LuaParameterDictionary lua_parameter_dictionary(
        code, std::move(file_resolver));
    return LoadOptions(&lua_parameter_dictionary);
}

}  // namespace system
}  // namespace autonomy