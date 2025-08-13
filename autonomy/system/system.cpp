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

#include "autonomy/system/system.hpp"

namespace autonomy {
namespace system { 


AutonomyNode::AutonomyNode(const proto::AutonomyOptions& options)
{
    
}

proto::AutonomyOptions CreateAutonomyOptions(common::LuaParameterDictionary* const parameter_dictionary)
{
    proto::AutonomyOptions options;
    // auto code = parameter_dictionary->GetDictionary("autonomy").get();

    *options.mutable_bridge_options() = bridge::CreateBridgeOptions(
        parameter_dictionary->GetDictionary("bridge").get());
    *options.mutable_controller_options() = control::CreateControllerOptions(
        parameter_dictionary->GetDictionary("control").get());
    *options.mutable_planner_options() = planning::CreatePlannerOptions(
        parameter_dictionary->GetDictionary("planning").get());
    *options.mutable_localization_options() = localization::CreateLocalizationOptions(
        parameter_dictionary->GetDictionary("localization").get());
    *options.mutable_map_options() = map::CreateMapOptions(
        parameter_dictionary->GetDictionary("map").get());
    *options.mutable_perception_options() = perception::CreatePerceptionOptions(
        parameter_dictionary->GetDictionary("perception").get());
    *options.mutable_task_options() = tasks::CreateTaskOptions(
        parameter_dictionary->GetDictionary("tasks").get());
    *options.mutable_transform_options() = transform::CreateTransformOptions(
        parameter_dictionary->GetDictionary("transform").get());

    // *options.mutable_task_options() = visualization::CreateVisualizationOptions(
    //     parameter_dictionary->GetDictionary("visualization").get());


    return options;
}

AutonomyNode::UniquePtr CreateAutonomyBuilder(const proto::AutonomyOptions& options)
{
    return std::make_unique<AutonomyNode>(options);
}


}   // namespace tasks
}   // namespace autonomy