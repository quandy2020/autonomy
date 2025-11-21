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

#include "autonomy/system/common/system_interface.hpp"
#include "autonomy/common/configuration_file_resolver.hpp"
#include "autonomy/common/json_util.hpp"
#include "autonomy/common/logging.hpp"

namespace autonomy {
namespace system { 
namespace common {

proto::AutonomyOptions CreateOptions(
    const std::string& configuration_directory,
    const std::string& configuration_basename)
{
    // Lua code
    auto file_resolver = std::make_unique<::autonomy::common::ConfigurationFileResolver>(
        std::vector<std::string>{configuration_directory});
    std::string code = file_resolver->GetFileContentOrDie(configuration_basename);
    ::autonomy::common::LuaParameterDictionary parameter_dictionary(code, std::move(file_resolver));

    // Options
    proto::AutonomyOptions options;
    *options.mutable_bridge_options() = bridge::common::LoadOptions(
        parameter_dictionary.GetDictionary("bridge").get());
    *options.mutable_controller_options() = control::common::LoadOptions(
        parameter_dictionary.GetDictionary("control").get());
    *options.mutable_localization_options() = localization::common::LoadOptions(
        parameter_dictionary.GetDictionary("localization").get());
    *options.mutable_map_options() = map::common::LoadOptions(
        parameter_dictionary.GetDictionary("map").get());
    *options.mutable_perception_options() = perception::common::LoadOptions(
        parameter_dictionary.GetDictionary("perception").get());
    *options.mutable_planner_options() = planning::common::LoadOptions(
        parameter_dictionary.GetDictionary("planning").get());
    *options.mutable_prediction_options() = prediction::common::LoadOptions(
        parameter_dictionary.GetDictionary("prediction").get());
    *options.mutable_task_options() = tasks::common::LoadOptions(
        parameter_dictionary.GetDictionary("tasks").get());
    *options.mutable_transform_options() = transform::common::LoadOptions(
        parameter_dictionary.GetDictionary("transform").get());
    *options.mutable_visualization_options() = visualization::common::LoadOptions(
        parameter_dictionary.GetDictionary("visualization").get());
    return options;
}

proto::AutonomyOptions LoadOptions(
    autonomy::common::LuaParameterDictionary* const parameter_dictionary)
{
    proto::AutonomyOptions options;
    return options;
}

}   // namespace common
}   // namespace system
}   // namespace autonomy