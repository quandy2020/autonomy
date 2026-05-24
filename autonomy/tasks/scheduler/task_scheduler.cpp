/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/scheduler/task_scheduler.hpp"

#include <algorithm>

#include "autonomy/common/configuration_file_resolver.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/common/lua_parameter_dictionary.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/control/control_options.hpp"
#include "autonomy/planning/planner_options.hpp"
#include "autonomy/tasks/common/behavior_tree_navigator.hpp"
#include "autonomy/tasks/constants.hpp"
#include "autonomy/tasks/navigator/docking/navigate_to_docking.hpp"
#include "autonomy/tasks/navigator/exploration/explore_to_anywhere.hpp"
#include "autonomy/tasks/navigator/navigation/navigate_through_poses.hpp"
#include "autonomy/tasks/navigator/navigation/navigate_to_pose.hpp"
#include "autonomy/tasks/navigator/navigator_factory.hpp"
#include "autonomy/tasks/navigator/teleop/teleop_drive.hpp"
#include "autonomy/tasks/navigator/teleop/teleop_session.hpp"
#include "autonomy/tasks/navigator/tracking/track_to_target.hpp"
#include "autonomy/tasks/options.hpp"
#include "autonomy/transform/buffer.hpp"

namespace autonomy {
namespace tasks {
namespace scheduler {
namespace {

planning::proto::PlannerOptions LoadPlannerOptions(
    const std::string& configuration_directory) {
    auto file_resolver =
        std::make_unique<::autonomy::common::ConfigurationFileResolver>(
            std::vector<std::string>{configuration_directory});
    const std::string code = ::autonomy::common::GetLuaScriptWithCommonOrDie(
        *file_resolver, "planner/planner.lua");
    ::autonomy::common::LuaParameterDictionary lua_dictionary(
        code, std::move(file_resolver));
    return planning::LoadOptions(
        lua_dictionary.GetDictionary("AUTONOMY_PLANNER").get());
}

control::proto::ControllerOptions LoadControllerOptions(
    const std::string& configuration_directory) {
    auto file_resolver =
        std::make_unique<::autonomy::common::ConfigurationFileResolver>(
            std::vector<std::string>{configuration_directory});
    const std::string code = ::autonomy::common::GetLuaScriptWithCommonOrDie(
        *file_resolver, "control/controller.lua");
    ::autonomy::common::LuaParameterDictionary lua_dictionary(
        code, std::move(file_resolver));
    return control::LoadOptions(
        lua_dictionary.GetDictionary("AUTONOMY_CONTROLLER").get());
}

std::string ResolveBehaviorTreePath(const std::string& configuration_directory,
                                    const std::string& filename) {
    return configuration_directory + "/tasks/behavior_tree/" + filename;
}

const proto::NavigatorConfig& NavigatorConfigFor(
    const proto::TaskOptions& options, const std::string& id) {
    if (id == "navigate_to_pose") {
        return options.navigate_to_pose();
    }
    if (id == "navigate_through_poses") {
        return options.navigate_through_poses();
    }
    if (id == "navigate_to_docking") {
        return options.navigate_to_docking();
    }
    if (id == "track_to_target") {
        return options.track_to_target();
    }
    if (id == "explore_to_anywhere") {
        return options.explore_to_anywhere();
    }
    if (id == "teleop_drive") {
        return options.teleop_drive();
    }
    static const proto::NavigatorConfig kEmpty{};
    return kEmpty;
}

}  // namespace

void TaskScheduler::ApplyTaskOptionsToContext() {
    if (!task_context_) {
        return;
    }
    task_context_->cancel_flag = &cancel_requested_;
    if (!task_options_.global_frame().empty()) {
        task_context_->global_frame = task_options_.global_frame();
    } else if (task_context_->global_frame.empty()) {
        task_context_->global_frame = "map";
    }
    if (!task_options_.robot_base_frame().empty()) {
        task_context_->robot_base_frame = task_options_.robot_base_frame();
    } else if (task_context_->robot_base_frame.empty()) {
        task_context_->robot_base_frame = "base_link";
    }
    if (!task_options_.default_planner_id().empty()) {
        task_context_->selected_planner_id = task_options_.default_planner_id();
    } else if (planner_) {
        task_context_->selected_planner_id = planner_->GetDefaultPlannerId();
    }
    if (planner_) {
        const auto planner_options = LoadPlannerOptions(configuration_directory_);
        if (!planner_options.default_smoother_id().empty()) {
            task_context_->selected_smoother_id =
                planner_options.default_smoother_id();
        } else if (smoother_) {
            task_context_->selected_smoother_id = smoother_->GetDefaultSmootherId();
        }
    }
    if (!task_options_.default_controller_id().empty()) {
        task_context_->selected_controller_id =
            task_options_.default_controller_id();
    }
    if (!task_options_.default_goal_checker_id().empty()) {
        task_context_->selected_goal_checker_id =
            task_options_.default_goal_checker_id();
    }
}

void TaskScheduler::Initialize(const std::string& configuration_directory) {
    if (initialized_) {
        return;
    }
    configuration_directory_ =
        ::autonomy::common::ResolveConfigurationRootDirectory(
            configuration_directory, "tasks/tasks.lua");
    if (!configuration_directory.empty() &&
        configuration_directory != configuration_directory_) {
        AWARN << "Configuration directory '" << configuration_directory
              << "' was not found; using '" << configuration_directory_ << "'.";
    }
    AINFO << "Using configuration directory: " << configuration_directory_;
    task_options_ =
        CreateOptions(configuration_directory_, "tasks/tasks.lua");

    const auto planner_options = LoadPlannerOptions(configuration_directory_);
    planner_ = std::make_shared<planning::PlannerServer>(planner_options);
    smoother_ = std::make_shared<planning::SmootherServer>(
        planner_options, planner_->GetCostmapWrapper());
    planner_->SetSmootherServer(smoother_);
    controller_ = std::make_shared<control::ControllerServer>(
        LoadControllerOptions(configuration_directory_));

    task_context_ = std::make_shared<common::TaskContext>();
    task_context_->planner = planner_;
    task_context_->smoother = smoother_;
    task_context_->controller = controller_;
    task_context_->global_costmap = planner_->GetCostmapWrapper();
    task_context_->local_costmap = controller_->GetCostmapWrapper();
    if (!task_context_->local_costmap) {
        task_context_->local_costmap = task_context_->global_costmap;
    }
    task_context_->tf = std::shared_ptr<transform::Buffer>(
        transform::Buffer::Instance(), [](transform::Buffer*) {});

    ApplyTaskOptionsToContext();

    controller_->SetNavigationContext(task_context_->tf, task_context_->global_frame,
                                      task_context_->robot_base_frame);
    controller_->SetSharedCostmap(planner_->GetCostmapWrapper());
    odom_smoother_ = controller_->GetOdomSmoother();
    task_context_->odom_smoother = odom_smoother_;

    planner_->Start();
    smoother_->Start();
    controller_->Start();

    SetupNavigators();
    owns_servers_ = true;
    initialized_ = true;
    AINFO << "TaskScheduler initialized (owned servers, " << navigators_.size()
          << " navigator(s)).";
}

void TaskScheduler::InitializeAttached(const std::string& configuration_directory,
                                       const SharedSystem& system) {
    if (initialized_) {
        return;
    }
    if (!system.planner || !system.controller || !system.task_context) {
        AERROR << "InitializeAttached requires planner, controller, and task_context.";
        return;
    }

    configuration_directory_ =
        ::autonomy::common::ResolveConfigurationRootDirectory(
            configuration_directory, "tasks/tasks.lua");
    if (!configuration_directory.empty() &&
        configuration_directory != configuration_directory_) {
        AWARN << "Configuration directory '" << configuration_directory
              << "' was not found; using '" << configuration_directory_ << "'.";
    }
    AINFO << "TaskScheduler attached to shared system (config="
          << configuration_directory_ << ")";
    task_options_ =
        CreateOptions(configuration_directory_, "tasks/tasks.lua");

    planner_ = system.planner;
    smoother_ = system.smoother;
    controller_ = system.controller;
    task_context_ = system.task_context;
    odom_smoother_ = controller_->GetOdomSmoother();
    if (odom_smoother_) {
        task_context_->odom_smoother = odom_smoother_;
    }

    ApplyTaskOptionsToContext();

    if (task_context_->tf && !task_context_->global_frame.empty()) {
        controller_->SetNavigationContext(
            task_context_->tf, task_context_->global_frame,
            task_context_->robot_base_frame.empty()
                ? "base_link"
                : task_context_->robot_base_frame);
    }
    if (task_context_->global_costmap) {
        controller_->SetSharedCostmap(task_context_->global_costmap);
    }

    SetupNavigators();
    owns_servers_ = false;
    initialized_ = true;
    AINFO << "TaskScheduler initialized (attached, " << navigators_.size()
          << " navigator(s)).";
}

void TaskScheduler::Shutdown() {
    if (!initialized_) {
        return;
    }
    navigators_.clear();
    if (owns_servers_) {
        if (controller_) {
            controller_->Shutdown();
        }
        if (smoother_) {
            smoother_->Shutdown();
        }
        if (planner_) {
            planner_->Shutdown();
        }
    }
    if (task_context_) {
        task_context_->cancel_flag = nullptr;
    }
    planner_.reset();
    smoother_.reset();
    controller_.reset();
    task_context_.reset();
    odom_smoother_.reset();
    initialized_ = false;
    owns_servers_ = false;
}

void TaskScheduler::SetupNavigators() {
    std::vector<std::string> plugin_libs;
    plugin_libs.reserve(task_options_.plugin_lib_names_size());
    for (const auto& name : task_options_.plugin_lib_names()) {
        plugin_libs.push_back(name);
    }

    common::FeedbackUtils feedback;
    feedback.global_frame = task_context_->global_frame;
    feedback.robot_frame = task_context_->robot_base_frame;
    feedback.tf = task_context_->tf;
    feedback.local_survival_timeout =
        task_options_.local_survival_timeout() > 0.0
            ? task_options_.local_survival_timeout()
            : 120.0;
    feedback.bt_xml_path_resolver =
        [this](const std::string& filename) {
            return ResolveBehaviorTreePath(configuration_directory_, filename);
        };

    auto muxer_alias = std::shared_ptr<common::NavigatorMuxer>(
        &muxer_, [](common::NavigatorMuxer*) {});

    navigator::NavigatorCreateContext ctx{
        task_options_, task_context_, plugin_libs, feedback, muxer_alias,
        odom_smoother_};

    for (const auto& id : task_options_.navigators()) {
        const auto& nav_cfg = NavigatorConfigFor(task_options_, id);
        if (!nav_cfg.enable()) {
            AINFO << "Navigator '" << id << "' disabled in config.";
            continue;
        }
        if (!navigator::NavigatorFactory::HasNavigator(id)) {
            AWARN << "Navigator '" << id
                  << "' is not registered in NavigatorFactory; skipping.";
            continue;
        }
        auto instance =
            navigator::NavigatorFactory::Create(id, ctx, nav_cfg);
        if (!instance) {
            AERROR << "NavigatorFactory failed to create '" << id << "'.";
            continue;
        }
        navigators_[id] = std::move(instance);
        AINFO << "Registered navigator '" << id << "' (BT: "
              << (nav_cfg.default_behavior_tree_file().empty()
                      ? "default"
                      : nav_cfg.default_behavior_tree_file())
              << ").";
    }
}

behavior_tree::BtStatus TaskScheduler::RunNavigator(
    const std::string& id,
    const std::function<behavior_tree::BtStatus(CancelFn)>& run) {
    if (!initialized_) {
        AERROR << "TaskScheduler not initialized.";
        return behavior_tree::BtStatus::FAILED;
    }
    if (!HasNavigator(id)) {
        AERROR << "Navigator '" << id << "' is disabled or not registered.";
        return behavior_tree::BtStatus::FAILED;
    }
    cancel_requested_.store(false);
    CancelFn cancel_fn = [this]() { return cancel_requested_.load(); };
    return run(cancel_fn);
}

bool TaskScheduler::HasNavigator(const std::string& id) const {
    return navigators_.find(id) != navigators_.end();
}

bool TaskScheduler::IsNavigatorConfigured(const std::string& id) const {
    const auto& cfg = NavigatorConfigFor(task_options_, id);
    if (id.empty()) {
        return false;
    }
    for (const auto& nav_id : task_options_.navigators()) {
        if (nav_id == id) {
            return cfg.enable();
        }
    }
    return false;
}

std::vector<std::string> TaskScheduler::RegisteredNavigatorIds() const {
    std::vector<std::string> ids;
    ids.reserve(navigators_.size());
    for (const auto& entry : navigators_) {
        ids.push_back(entry.first);
    }
    std::sort(ids.begin(), ids.end());
    return ids;
}

behavior_tree::BtStatus TaskScheduler::NavigateToPose(
    std::shared_ptr<const behavior_tree::proto::NavigateToPoseAction::Goal>
        goal) {
    return RunNavigator(kNavigatorNavigateToPose,
        [&](CancelFn cancel) {
            auto navigate_to_pose =
                GetNavigator<navigator::navigation::NavigateToPoseNavigator>(
                    kNavigatorNavigateToPose);
            return navigate_to_pose->Bt().Run(goal, cancel);
        });
}

behavior_tree::BtStatus TaskScheduler::NavigateThroughPoses(
    std::shared_ptr<const behavior_tree::proto::NavigateThroughPosesAction::Goal>
        goal) {
    return RunNavigator(kNavigatorNavigateThroughPoses,
        [&](CancelFn cancel) {
            auto nav =
                GetNavigator<navigator::navigation::NavigateThroughPosesNavigator>(
                    kNavigatorNavigateThroughPoses);
            return nav->Bt().Run(goal, cancel);
        });
}

behavior_tree::BtStatus TaskScheduler::NavigateThroughPoses(
    const std::vector<commsgs::geometry_msgs::PoseStamped>& poses,
    const std::string& behavior_tree) {
    if (poses.empty()) {
        AERROR << "NavigateThroughPoses requires at least one pose.";
        return behavior_tree::BtStatus::FAILED;
    }
    auto goal = std::make_shared<
        behavior_tree::proto::NavigateThroughPosesAction::Goal>();
    for (const auto& pose : poses) {
        *goal->mutable_poses()->add_goals() =
            commsgs::geometry_msgs::ToProto(pose);
    }
    if (!behavior_tree.empty()) {
        goal->set_behavior_tree(behavior_tree);
    }
    return NavigateThroughPoses(goal);
}

behavior_tree::BtStatus TaskScheduler::NavigateToDock(
    std::shared_ptr<const behavior_tree::proto::DockRobotAction::Goal> goal) {
    return RunNavigator(kNavigatorNavigateToDocking,
        [&](CancelFn cancel) {
            auto nav = GetNavigator<navigator::docking::NavigateToDockingNavigator>(
                kNavigatorNavigateToDocking);
            return nav->Bt().Run(goal, cancel);
        });
}

behavior_tree::BtStatus TaskScheduler::NavigateToDockPose(
    const commsgs::geometry_msgs::PoseStamped& dock_pose,
    const std::string& dock_type, const std::string& dock_id) {
    auto goal = std::make_shared<behavior_tree::proto::DockRobotAction::Goal>();
    goal->set_use_dock_id(!dock_id.empty());
    goal->set_dock_id(dock_id);
    goal->set_dock_type(dock_type);
    goal->set_navigate_to_staging_pose(true);
    *goal->mutable_dock_pose() = commsgs::geometry_msgs::ToProto(dock_pose);
    return NavigateToDock(goal);
}

behavior_tree::BtStatus TaskScheduler::TrackToTarget(
    std::shared_ptr<const behavior_tree::proto::TrackToTargetAction::Goal>
        goal) {
    return RunNavigator(kNavigatorTrackToTarget,
        [&](CancelFn cancel) {
            auto nav = GetNavigator<navigator::tracking::TrackToTargetNavigator>(
                kNavigatorTrackToTarget);
            return nav->Bt().Run(goal, cancel);
        });
}

behavior_tree::BtStatus TaskScheduler::TrackToTarget(const uint32_t target_id) {
    auto goal = std::make_shared<
        behavior_tree::proto::TrackToTargetAction::Goal>();
    goal->set_target_id(target_id);
    return TrackToTarget(goal);
}

bool TaskScheduler::UpdateTrackTargetPose(
    const commsgs::geometry_msgs::PoseStamped& target_pose) {
    auto nav = GetNavigator<navigator::tracking::TrackToTargetNavigator>(
        kNavigatorTrackToTarget);
    if (!nav) {
        return false;
    }
    nav->UpdateTargetPose(target_pose);
    return true;
}

behavior_tree::BtStatus TaskScheduler::ExploreToAnywhere(
    std::shared_ptr<const behavior_tree::proto::ExploreToAnywhereAction::Goal>
        goal) {
    return RunNavigator(kNavigatorExploreToAnywhere,
        [&](CancelFn cancel) {
            auto nav =
                GetNavigator<navigator::exploration::ExploreToAnywhereNavigator>(
                    kNavigatorExploreToAnywhere);
            return nav->Bt().Run(goal, cancel);
        });
}

behavior_tree::BtStatus TaskScheduler::ExploreToAnywhere(
    const double time_allowance_sec) {
    auto goal = std::make_shared<
        behavior_tree::proto::ExploreToAnywhereAction::Goal>();
    if (time_allowance_sec > 0.0) {
        *goal->mutable_time_allowance() = commsgs::builtin_interfaces::ToProto(
            commsgs::builtin_interfaces::Duration::FromSeconds(
                time_allowance_sec));
    }
    return ExploreToAnywhere(goal);
}

bool TaskScheduler::UpdateExploreGoal(
    const commsgs::geometry_msgs::PoseStamped& explore_goal) {
    auto nav = GetNavigator<navigator::exploration::ExploreToAnywhereNavigator>(
        kNavigatorExploreToAnywhere);
    if (!nav) {
        return false;
    }
    nav->UpdateExploreGoal(explore_goal);
    return true;
}

behavior_tree::BtStatus TaskScheduler::TeleopDrive(
    std::shared_ptr<const behavior_tree::proto::AssistedTeleopAction::Goal> goal,
    const double max_linear_vel, const double max_angular_vel,
    std::function<bool()> cancel_checker) {
    if (!initialized_ || !goal) {
        return behavior_tree::BtStatus::FAILED;
    }
    return RunNavigator(kNavigatorTeleopDrive,
        [&, checker = std::move(cancel_checker)](CancelFn cancel) {
            auto nav = GetNavigator<navigator::teleop::TeleopDriveNavigator>(
                kNavigatorTeleopDrive);
            if (!nav) {
                return behavior_tree::BtStatus::FAILED;
            }
            nav->setRunLimits(max_linear_vel, max_angular_vel);
            CancelFn merged_cancel = [cancel, &checker]() {
                return cancel() || (checker && checker());
            };
            return nav->Bt().Run(goal, merged_cancel);
        });
}

behavior_tree::BtStatus TaskScheduler::TeleopDrive(
    const double time_allowance_sec,
    const double max_linear_vel, const double max_angular_vel,
    std::function<bool()> cancel_checker) {
    auto goal = std::make_shared<
        behavior_tree::proto::AssistedTeleopAction::Goal>();
    if (time_allowance_sec > 0.0) {
        *goal->mutable_time_allowance() = commsgs::builtin_interfaces::ToProto(
            commsgs::builtin_interfaces::Duration::FromSeconds(
                time_allowance_sec));
    }
    return TeleopDrive(goal, max_linear_vel, max_angular_vel, std::move(cancel_checker));
}

bool TaskScheduler::UpdateTeleopCommand(
    const commsgs::geometry_msgs::TwistStamped& cmd) {
    auto nav = GetNavigator<navigator::teleop::TeleopDriveNavigator>(
        kNavigatorTeleopDrive);
    if (!nav || !nav->session()) {
        return false;
    }
    nav->session()->UpdateCommand(cmd);
    return true;
}

void TaskScheduler::BeginTeleopSession(
    const double max_linear_vel, const double max_angular_vel) {
    auto nav = GetNavigator<navigator::teleop::TeleopDriveNavigator>(
        kNavigatorTeleopDrive);
    if (!nav || !nav->session()) {
        return;
    }
    navigator::teleop::TeleopSession::Limits limits;
    if (task_options_.has_teleop_drive_options()) {
        const auto& opts = task_options_.teleop_drive_options();
        limits.max_linear_vel = opts.default_max_linear_vel();
        limits.max_angular_vel = opts.default_max_angular_vel();
        limits.cmd_stale_timeout_sec = opts.cmd_stale_timeout_sec();
    }
    if (max_linear_vel > 0.0) {
        limits.max_linear_vel = max_linear_vel;
    }
    if (max_angular_vel > 0.0) {
        limits.max_angular_vel = max_angular_vel;
    }
    nav->session()->Begin(0.0, limits);
}

void TaskScheduler::EndTeleopSession() {
    auto nav = GetNavigator<navigator::teleop::TeleopDriveNavigator>(
        kNavigatorTeleopDrive);
    if (nav && nav->session()) {
        nav->session()->End();
    }
}

bool TaskScheduler::IsTeleopActive() const {
    auto nav = GetNavigator<navigator::teleop::TeleopDriveNavigator>(
        kNavigatorTeleopDrive);
    return nav && nav->session() && nav->session()->IsActive();
}

void TaskScheduler::RequestCancel() {
    cancel_requested_.store(true);
    for (auto& entry : navigators_) {
        if (entry.second) {
            entry.second->RequestCancel();
        }
    }
}

}  // namespace scheduler
}  // namespace tasks
}  // namespace autonomy
