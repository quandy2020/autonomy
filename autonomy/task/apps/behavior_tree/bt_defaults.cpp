/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/task/apps/behavior_tree/bt_defaults.hpp"

#include "autonomy/common/config.hpp"

namespace autonomy {
namespace task {
namespace {

using RobotTaskType = ::autonomy::commsgs::proto::vehicle_msgs::RobotTaskType;
namespace proto = ::autonomy::task::proto;

const proto::TaskBehaviorTreeProfile* ProtoProfile(
    const proto::TaskServerOptions& options, RobotTaskType type)
{
    if (!options.has_behavior_trees()) {
        return nullptr;
    }
    const auto& trees = options.behavior_trees();
    switch (type) {
    case RobotTaskType::ROBOT_TASK_NAVIGATION:
        return trees.has_navigation() ? &trees.navigation() : nullptr;
    case RobotTaskType::ROBOT_TASK_FOLLOW:
        return trees.has_tracking() ? &trees.tracking() : nullptr;
    case RobotTaskType::ROBOT_TASK_TELEOP:
        return trees.has_teleop() ? &trees.teleop() : nullptr;
    case RobotTaskType::ROBOT_TASK_EXPLORATION:
        return trees.has_exploration() ? &trees.exploration() : nullptr;
    case RobotTaskType::ROBOT_TASK_DOCK:
        return trees.has_charging() ? &trees.charging() : nullptr;
    case RobotTaskType::ROBOT_TASK_MAP:
        return trees.has_mapping() ? &trees.mapping() : nullptr;
    default:
        return trees.has_localization() ? &trees.localization() : nullptr;
    }
}

void SetProfile(proto::TaskBehaviorTreeProfile* profile,
                const char* default_tree,
                const char* alternate_tree,
                uint32_t loop_period_ms,
                std::initializer_list<const char*> plugins)
{
    profile->set_default_tree_file(default_tree);
    if (alternate_tree != nullptr) {
        profile->set_alternate_tree_file(alternate_tree);
    }
    profile->set_bt_loop_duration_ms(loop_period_ms);
    profile->set_default_server_timeout_ms(20000);
    profile->set_plugin_lib_path(
        std::string(autonomy::common::kLibraryInstallDir) + "/lib");
    for (const char* name : plugins) {
        profile->add_plugin_lib_names(name);
    }
}

}  // namespace

BtProfile BtDefaults::ProfileFor(const proto::TaskServerOptions& options,
                                 RobotTaskType type)
{
    if (const auto* proto_profile = ProtoProfile(options, type)) {
        return BtProfile::FromProto(*proto_profile);
    }
    return {};
}

void BtDefaults::Apply(proto::TaskServerOptions* options)
{
    if (options == nullptr) {
        return;
    }
    if (options->config_directory().empty()) {
        options->set_config_directory("config");
    }

    auto* bt = options->mutable_behavior_trees();

    SetProfile(bt->mutable_navigation(),
               "task/behavior_tree/navigation/navigate_to_pose.xml",
               "task/behavior_tree/navigation/navigate_through_poses.xml", 10,
               {"autonomy_task_navigation_action_compute_path_action",
                "autonomy_task_navigation_action_smooth_path_action",
                "autonomy_task_navigation_action_follow_path_action",
                "autonomy_task_navigation_action_motion_actions",
                "autonomy_task_navigation_action_costmap_actions",
                "autonomy_task_navigation_condition_goal_reached_condition",
                "autonomy_task_navigation_condition_servers_ready_condition",
                "autonomy_task_navigation_condition_path_valid_condition"});

    SetProfile(bt->mutable_tracking(),
               "task/behavior_tree/tracking/follow_target.xml",
               "task/behavior_tree/tracking/follow_person.xml", 20,
               {"autonomy_task_tracking_action_compute_follow_goal_action",
                "autonomy_task_tracking_condition_target_locked_condition",
                "autonomy_task_navigation_action_compute_path_action",
                "autonomy_task_navigation_action_smooth_path_action",
                "autonomy_task_navigation_action_follow_path_action",
                "autonomy_task_navigation_condition_servers_ready_condition",
                "autonomy_task_navigation_condition_path_valid_condition",
                "autonomy_task_navigation_condition_goal_reached_condition"});

    SetProfile(bt->mutable_teleop(), "task/behavior_tree/teleop/teleop.xml",
               nullptr, 50,
               {"autonomy_task_teleop_action_apply_teleop_velocity_action",
                "autonomy_task_teleop_condition_teleop_watchdog_ok_condition"});

    SetProfile(bt->mutable_exploration(),
               "task/behavior_tree/exploration/explore.xml", nullptr, 10,
               {"autonomy_task_exploration_action_select_frontier_action",
                "autonomy_task_exploration_action_save_exploration_map_action",
                "autonomy_task_exploration_condition_frontier_available_condition",
                "autonomy_task_navigation_action_compute_path_action",
                "autonomy_task_navigation_action_smooth_path_action",
                "autonomy_task_navigation_action_follow_path_action",
                "autonomy_task_navigation_condition_servers_ready_condition",
                "autonomy_task_navigation_condition_path_valid_condition",
                "autonomy_task_navigation_condition_goal_reached_condition"});

    SetProfile(bt->mutable_charging(), "task/behavior_tree/charging/dock.xml",
               nullptr, 10,
               {"autonomy_task_charging_action_dock_search_action",
                "autonomy_task_charging_action_dock_connect_action",
                "autonomy_task_navigation_action_compute_path_action",
                "autonomy_task_navigation_action_smooth_path_action",
                "autonomy_task_navigation_action_follow_path_action",
                "autonomy_task_navigation_condition_servers_ready_condition",
                "autonomy_task_navigation_condition_path_valid_condition"});

    SetProfile(bt->mutable_mapping(),
               "task/behavior_tree/mapping/map_load.xml",
               "task/behavior_tree/mapping/map_set_pose.xml", 100,
               {"autonomy_task_mapping_action_load_map_action",
                "autonomy_task_mapping_action_set_initial_pose_action",
                "autonomy_task_mapping_action_clear_costmap_action",
                "autonomy_task_navigation_action_costmap_actions"});

    SetProfile(bt->mutable_localization(),
               "task/behavior_tree/localization/localization.xml", nullptr, 100,
               {"autonomy_task_localization_action_start_localization_action",
                "autonomy_task_localization_action_stop_localization_action"});
}

}  // namespace task
}  // namespace autonomy
