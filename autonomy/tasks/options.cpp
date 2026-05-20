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

#include "autonomy/tasks/options.hpp"

#include "autonomy/common/configuration_file_resolver.hpp"
#include "autonomy/common/lua_parameter_dictionary.hpp"

namespace autonomy {
namespace tasks {
namespace {

proto::NavigatorConfig LoadNavigatorConfig(
    ::autonomy::common::LuaParameterDictionary* const parameter_dictionary) {
    proto::NavigatorConfig config;
    // Read keys in same order as tasks.lua sub-tables (enable, plugin,
    // default_behavior_tree_file, etc.)
    if (parameter_dictionary->HasKey("enable")) {
        config.set_enable(parameter_dictionary->GetBool("enable"));
    }
    if (parameter_dictionary->HasKey("plugin")) {
        config.set_plugin(parameter_dictionary->GetString("plugin"));
    }
    if (parameter_dictionary->HasKey("enable_groot_monitoring")) {
        config.set_enable_groot_monitoring(
            parameter_dictionary->GetBool("enable_groot_monitoring"));
    }
    if (parameter_dictionary->HasKey("groot_server_port")) {
        config.set_groot_server_port(
            parameter_dictionary->GetInt("groot_server_port"));
    }
    if (parameter_dictionary->HasKey("default_behavior_tree_file")) {
        config.set_default_behavior_tree_file(
            parameter_dictionary->GetString("default_behavior_tree_file"));
    }

    return config;
}

proto::TaskOptions LoadTaskOptions(
    ::autonomy::common::LuaParameterDictionary* const parameter_dictionary) {
    proto::TaskOptions options;

    if (parameter_dictionary->HasKey("global_frame")) {
        options.set_global_frame(
            parameter_dictionary->GetString("global_frame"));
    }
    if (parameter_dictionary->HasKey("robot_base_frame")) {
        options.set_robot_base_frame(
            parameter_dictionary->GetString("robot_base_frame"));
    }
    if (parameter_dictionary->HasKey("odom_topic")) {
        options.set_odom_topic(parameter_dictionary->GetString("odom_topic"));
    }
    if (parameter_dictionary->HasKey("bt_loop_duration")) {
        options.set_bt_loop_duration(
            parameter_dictionary->GetInt("bt_loop_duration"));
    }
    if (parameter_dictionary->HasKey("filter_duration")) {
        options.set_filter_duration(
            parameter_dictionary->GetDouble("filter_duration"));
    }
    if (parameter_dictionary->HasKey("default_server_timeout")) {
        options.set_default_server_timeout(
            parameter_dictionary->GetInt("default_server_timeout"));
    }
    if (parameter_dictionary->HasKey("wait_for_service_timeout")) {
        options.set_wait_for_service_timeout(
            parameter_dictionary->GetInt("wait_for_service_timeout"));
    }
    if (parameter_dictionary->HasKey("service_introspection_mode")) {
        options.set_service_introspection_mode(
            parameter_dictionary->GetString("service_introspection_mode"));
    }

    if (parameter_dictionary->HasKey("navigators")) {
        auto navigators_dict =
            parameter_dictionary->GetDictionary("navigators");
        auto navigators = navigators_dict->GetArrayValuesAsStrings();
        for (const auto& navigator : navigators) {
            options.add_navigators(navigator);
        }
    }

    if (parameter_dictionary->HasKey("navigate_to_pose")) {
        auto navigate_to_pose_dict =
            parameter_dictionary->GetDictionary("navigate_to_pose");
        *options.mutable_navigate_to_pose() =
            LoadNavigatorConfig(navigate_to_pose_dict.get());
    }

    if (parameter_dictionary->HasKey("navigate_through_poses")) {
        auto navigate_through_poses_dict =
            parameter_dictionary->GetDictionary("navigate_through_poses");
        *options.mutable_navigate_through_poses() =
            LoadNavigatorConfig(navigate_through_poses_dict.get());
    }

    if (parameter_dictionary->HasKey("navigate_to_docking")) {
        auto navigate_to_docking_dict =
            parameter_dictionary->GetDictionary("navigate_to_docking");
        *options.mutable_navigate_to_docking() =
            LoadNavigatorConfig(navigate_to_docking_dict.get());
    }

    if (parameter_dictionary->HasKey("track_to_target")) {
        auto track_to_target_dict =
            parameter_dictionary->GetDictionary("track_to_target");
        *options.mutable_track_to_target() =
            LoadNavigatorConfig(track_to_target_dict.get());
    }

    if (parameter_dictionary->HasKey("explore_to_anywhere")) {
        auto explore_to_anywhere_dict =
            parameter_dictionary->GetDictionary("explore_to_anywhere");
        *options.mutable_explore_to_anywhere() =
            LoadNavigatorConfig(explore_to_anywhere_dict.get());
    }

    if (parameter_dictionary->HasKey("error_code_name_prefixes")) {
        auto error_code_prefixes_dict =
            parameter_dictionary->GetDictionary("error_code_name_prefixes");
        auto error_code_prefixes =
            error_code_prefixes_dict->GetArrayValuesAsStrings();
        for (const auto& prefix : error_code_prefixes) {
            options.add_error_code_name_prefixes(prefix);
        }
    }

    if (parameter_dictionary->HasKey("plugin_index_path")) {
        options.set_plugin_index_path(
            parameter_dictionary->GetString("plugin_index_path"));
    }
    if (parameter_dictionary->HasKey("plugin_lib_path")) {
        options.set_plugin_lib_path(
            parameter_dictionary->GetString("plugin_lib_path"));
    }
    if (parameter_dictionary->HasKey("plugin_lib_names")) {
        auto plugin_lib_names_dict =
            parameter_dictionary->GetDictionary("plugin_lib_names");
        auto names = plugin_lib_names_dict->GetArrayValuesAsStrings();
        for (const auto& name : names) {
            options.add_plugin_lib_names(name);
        }
    }
    if (parameter_dictionary->HasKey("enable_groot_monitoring")) {
        options.set_enable_groot_monitoring(
            parameter_dictionary->GetBool("enable_groot_monitoring"));
    }
    if (parameter_dictionary->HasKey("groot_server_port")) {
        options.set_groot_server_port(
            parameter_dictionary->GetInt("groot_server_port"));
    }
    if (parameter_dictionary->HasKey("local_survival_timeout")) {
        options.set_local_survival_timeout(
            parameter_dictionary->GetDouble("local_survival_timeout"));
    }
    if (parameter_dictionary->HasKey("plugin_profile")) {
        options.set_plugin_profile(
            parameter_dictionary->GetString("plugin_profile"));
    }
    if (parameter_dictionary->HasKey("default_planner_id")) {
        options.set_default_planner_id(
            parameter_dictionary->GetString("default_planner_id"));
    }
    if (parameter_dictionary->HasKey("default_controller_id")) {
        options.set_default_controller_id(
            parameter_dictionary->GetString("default_controller_id"));
    }
    if (parameter_dictionary->HasKey("default_goal_checker_id")) {
        options.set_default_goal_checker_id(
            parameter_dictionary->GetString("default_goal_checker_id"));
    }
    if (parameter_dictionary->HasKey("goal_reached_tolerance")) {
        options.set_goal_reached_tolerance(
            parameter_dictionary->GetDouble("goal_reached_tolerance"));
    }
    if (parameter_dictionary->HasKey("plugin_lib_names_minimal")) {
        auto dict =
            parameter_dictionary->GetDictionary("plugin_lib_names_minimal");
        for (const auto& name : dict->GetArrayValuesAsStrings()) {
            options.add_plugin_lib_names_minimal(name);
        }
    }
    if (parameter_dictionary->HasKey("plugin_lib_names_full")) {
        auto dict =
            parameter_dictionary->GetDictionary("plugin_lib_names_full");
        for (const auto& name : dict->GetArrayValuesAsStrings()) {
            options.add_plugin_lib_names_full(name);
        }
    }

    // Resolve plugin_lib_names: explicit list wins, else profile-based list.
    if (options.plugin_lib_names().empty()) {
        const bool use_full = options.plugin_profile() == "full";
        const auto& profile_list =
            use_full ? options.plugin_lib_names_full()
                     : options.plugin_lib_names_minimal();
        for (const auto& name : profile_list) {
            options.add_plugin_lib_names(name);
        }
    }

    return options;
}

}  // namespace

proto::TaskOptions LoadOptions(
    ::autonomy::common::LuaParameterDictionary* const parameter_dictionary) {
    if (parameter_dictionary->HasKey("tasks")) {
        auto tasks_dict = parameter_dictionary->GetDictionary("tasks");
        return LoadTaskOptions(tasks_dict.get());
    }
    return proto::TaskOptions{};
}

proto::TaskOptions CreateOptions(const std::string& configuration_directory,
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

}  // namespace tasks
}  // namespace autonomy
