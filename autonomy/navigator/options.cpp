/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/navigator/options.hpp"

#include "autonomy/common/configuration_file_resolver.hpp"
#include "autonomy/common/lua_parameter_dictionary.hpp"

namespace autonomy {
namespace navigator {
namespace {

void LoadBehaviorTreeNavigatorOptions(
    ::autonomy::common::LuaParameterDictionary* const dict,
    proto::BehaviorTreeNavigatorOptions* navigator) {
    if (dict->HasKey("enable")) {
        navigator->set_enable(dict->GetBool("enable"));
    }
    if (dict->HasKey("behavior_tree_file")) {
        navigator->set_behavior_tree_file(
            dict->GetString("behavior_tree_file"));
    } else if (dict->HasKey("default_behavior_tree_file")) {
        navigator->set_behavior_tree_file(
            dict->GetString("default_behavior_tree_file"));
    }
}

proto::NavigatorOptions LoadNavigatorOptions(
    ::autonomy::common::LuaParameterDictionary* const parameter_dictionary) {
    proto::NavigatorOptions options;

    if (parameter_dictionary->HasKey("global_frame")) {
        options.set_global_frame(
            parameter_dictionary->GetString("global_frame"));
    }
    if (parameter_dictionary->HasKey("robot_base_frame")) {
        options.set_robot_base_frame(
            parameter_dictionary->GetString("robot_base_frame"));
    }
    if (parameter_dictionary->HasKey("bt_loop_duration")) {
        options.set_bt_loop_duration(
            parameter_dictionary->GetInt("bt_loop_duration"));
    }
    if (parameter_dictionary->HasKey("default_server_timeout")) {
        options.set_default_server_timeout(
            parameter_dictionary->GetInt("default_server_timeout"));
    }
    if (parameter_dictionary->HasKey("local_survival_timeout")) {
        options.set_local_survival_timeout(
            parameter_dictionary->GetDouble("local_survival_timeout"));
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
    if (parameter_dictionary->HasKey("default_smoother_id")) {
        options.set_default_smoother_id(
            parameter_dictionary->GetString("default_smoother_id"));
    }
    if (parameter_dictionary->HasKey("goal_reached_tolerance")) {
        options.set_goal_reached_tolerance(
            parameter_dictionary->GetDouble("goal_reached_tolerance"));
    }
    if (parameter_dictionary->HasKey("plugin_lib_path")) {
        options.set_plugin_lib_path(
            parameter_dictionary->GetString("plugin_lib_path"));
    }
    if (parameter_dictionary->HasKey("plugin_lib_names")) {
        auto plugin_lib_names_dict =
            parameter_dictionary->GetDictionary("plugin_lib_names");
        for (const auto& name :
             plugin_lib_names_dict->GetArrayValuesAsStrings()) {
            options.add_plugin_lib_names(name);
        }
    }
    if (parameter_dictionary->HasKey("navigate_to_pose")) {
        LoadBehaviorTreeNavigatorOptions(
            parameter_dictionary->GetDictionary("navigate_to_pose").get(),
            options.mutable_navigate_to_pose());
    }
    if (parameter_dictionary->HasKey("navigate_through_poses")) {
        LoadBehaviorTreeNavigatorOptions(
            parameter_dictionary->GetDictionary("navigate_through_poses").get(),
            options.mutable_navigate_through_poses());
    }
    if (parameter_dictionary->HasKey("enable_autolink_action_servers")) {
        options.set_enable_autolink_action_servers(
            parameter_dictionary->GetBool("enable_autolink_action_servers"));
    }

    return options;
}

}  // namespace

proto::NavigatorOptions LoadOptions(
    ::autonomy::common::LuaParameterDictionary* const parameter_dictionary) {
    if (parameter_dictionary->HasKey("navigator")) {
        auto navigator_dict = parameter_dictionary->GetDictionary("navigator");
        return LoadNavigatorOptions(navigator_dict.get());
    }
    return proto::NavigatorOptions{};
}

proto::NavigatorOptions CreateOptions(
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

}  // namespace navigator
}  // namespace autonomy
