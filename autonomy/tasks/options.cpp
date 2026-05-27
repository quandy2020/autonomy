/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/options.hpp"

#include "autonomy/common/configuration_file_resolver.hpp"
#include "autonomy/common/lua_parameter_dictionary.hpp"

namespace autonomy {
namespace tasks {
namespace {

void LoadNavigateToPoseNavigator(
    ::autonomy::common::LuaParameterDictionary* const dict,
    proto::NavigateToPoseNavigator* navigator) {
    if (dict->HasKey("enable")) {
        navigator->set_enable(dict->GetBool("enable"));
    }
    if (dict->HasKey("default_behavior_tree_file")) {
        navigator->set_default_behavior_tree_file(
            dict->GetString("default_behavior_tree_file"));
    }
    if (dict->HasKey("goal_blackboard_key")) {
        navigator->set_goal_blackboard_key(dict->GetString("goal_blackboard_key"));
    }
    if (dict->HasKey("path_blackboard_key")) {
        navigator->set_path_blackboard_key(dict->GetString("path_blackboard_key"));
    }
}

void LoadNavigateThroughPosesNavigator(
    ::autonomy::common::LuaParameterDictionary* const dict,
    proto::NavigateThroughPosesNavigator* navigator) {
    if (dict->HasKey("enable")) {
        navigator->set_enable(dict->GetBool("enable"));
    }
    if (dict->HasKey("default_behavior_tree_file")) {
        navigator->set_default_behavior_tree_file(
            dict->GetString("default_behavior_tree_file"));
    }
    if (dict->HasKey("goals_blackboard_key")) {
        navigator->set_goals_blackboard_key(
            dict->GetString("goals_blackboard_key"));
    }
    if (dict->HasKey("path_blackboard_key")) {
        navigator->set_path_blackboard_key(dict->GetString("path_blackboard_key"));
    }
}

}  // namespace

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
    if (parameter_dictionary->HasKey("navigators")) {
        auto navigators_dict =
            parameter_dictionary->GetDictionary("navigators");
        for (const auto& navigator :
             navigators_dict->GetArrayValuesAsStrings()) {
            options.add_navigators(navigator);
        }
    }
    if (parameter_dictionary->HasKey("navigate_to_pose")) {
        LoadNavigateToPoseNavigator(
            parameter_dictionary->GetDictionary("navigate_to_pose").get(),
            options.mutable_navigate_to_pose());
    }
    if (parameter_dictionary->HasKey("navigate_through_poses")) {
        LoadNavigateThroughPosesNavigator(
            parameter_dictionary->GetDictionary("navigate_through_poses").get(),
            options.mutable_navigate_through_poses());
    }

    if (parameter_dictionary->HasKey("enable_autolink_action_servers")) {
        options.set_enable_autolink_action_servers(
            parameter_dictionary->GetBool("enable_autolink_action_servers"));
    }

    return options;
}

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
