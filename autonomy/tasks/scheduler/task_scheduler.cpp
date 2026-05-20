/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/tasks/scheduler/task_scheduler.hpp"

#include <algorithm>

#include "autonomy/common/configuration_file_resolver.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/common/lua_parameter_dictionary.hpp"
#include "autonomy/control/control_options.hpp"
#include "autonomy/planning/planner_options.hpp"
#include "autonomy/tasks/common/behavior_tree_navigator.hpp"
#include "autonomy/tasks/navigator/navigator_factory.hpp"
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
    const std::string code =
        file_resolver->GetFileContentOrDie("planner/planner.lua");
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
    const std::string code =
        file_resolver->GetFileContentOrDie("control/controller.lua");
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
    static const proto::NavigatorConfig kEmpty{};
    return kEmpty;
}

}  // namespace

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
    task_context_->cancel_flag = &cancel_requested_;
    task_context_->global_frame = task_options_.global_frame().empty()
                                      ? "map"
                                      : task_options_.global_frame();
    task_context_->robot_base_frame =
        task_options_.robot_base_frame().empty()
            ? "base_link"
            : task_options_.robot_base_frame();
    if (!task_options_.default_planner_id().empty()) {
        task_context_->selected_planner_id = task_options_.default_planner_id();
    } else if (planner_) {
        task_context_->selected_planner_id = planner_->GetDefaultPlannerId();
    }
    if (!planner_options.default_smoother_id().empty()) {
        task_context_->selected_smoother_id =
            planner_options.default_smoother_id();
    } else if (smoother_) {
        task_context_->selected_smoother_id = smoother_->GetDefaultSmootherId();
    }
    if (!task_options_.default_controller_id().empty()) {
        task_context_->selected_controller_id =
            task_options_.default_controller_id();
    }
    if (!task_options_.default_goal_checker_id().empty()) {
        task_context_->selected_goal_checker_id =
            task_options_.default_goal_checker_id();
    }
    controller_->SetNavigationContext(task_context_->tf, task_context_->global_frame,
                                      task_context_->robot_base_frame);
    controller_->SetSharedCostmap(planner_->GetCostmapWrapper());
    task_context_->odom_smoother =
        std::make_shared<control::utils::OdomSmoother>(
            std::max(0.0, task_options_.filter_duration()));
    odom_smoother_ = task_context_->odom_smoother;
    controller_->SetOdomSmoother(task_context_->odom_smoother);

    planner_->Start();
    smoother_->Start();
    controller_->Start();

    SetupNavigators();
    initialized_ = true;
    AINFO << "TaskScheduler initialized (single-process BT, "
          << navigators_.size() << " navigator(s)).";
}

void TaskScheduler::Shutdown() {
    if (!initialized_) {
        return;
    }
    navigators_.clear();
    if (controller_) {
        controller_->Shutdown();
    }
    if (smoother_) {
        smoother_->Shutdown();
    }
    if (planner_) {
        planner_->Shutdown();
    }
    initialized_ = false;
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

behavior_tree::BtStatus TaskScheduler::NavigateToPose(
    std::shared_ptr<const behavior_tree::proto::NavigateToPoseAction::Goal>
        goal) {
    auto navigate_to_pose =
        GetNavigator<navigator::navigation::NavigateToPoseNavigator>(
            "navigate_to_pose");
    if (!navigate_to_pose) {
        AERROR << "NavigateToPose navigator is disabled or not registered.";
        return behavior_tree::BtStatus::FAILED;
    }
    cancel_requested_.store(false);
    return navigate_to_pose->Bt().Run(
        goal, [this]() { return cancel_requested_.load(); });
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
