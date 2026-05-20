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

#include "autonomy/tasks/scheduler/task_scheduler.hpp"

#include "autonomy/common/configuration_file_resolver.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/common/lua_parameter_dictionary.hpp"
#include "autonomy/control/control_options.hpp"
#include "autonomy/planning/planner_options.hpp"
#include "autonomy/tasks/common/behavior_tree_navigator.hpp"
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

    planner_ = std::make_shared<planning::PlannerServer>(
        LoadPlannerOptions(configuration_directory_));
    controller_ = std::make_shared<control::ControllerServer>(
        LoadControllerOptions(configuration_directory_));

    planner_->Start();
    controller_->Start();

    task_context_ = std::make_shared<common::TaskContext>();
    task_context_->planner = planner_;
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
    controller_->SetNavigationContext(task_context_->tf, task_context_->global_frame,
                                      task_context_->robot_base_frame);
    if (task_options_.filter_duration() > 0.0) {
        task_context_->odom_smoother =
            std::make_shared<control::utils::OdomSmoother>(
                task_options_.filter_duration());
        odom_smoother_ = task_context_->odom_smoother;
    }

    SetupNavigators();
    initialized_ = true;
    AINFO << "TaskScheduler initialized (single-process BT).";
}

void TaskScheduler::Shutdown() {
    if (!initialized_) {
        return;
    }
    if (controller_) {
        controller_->Shutdown();
    }
    if (planner_) {
        planner_->Shutdown();
    }
    initialized_ = false;
}

void TaskScheduler::SetupNavigators() {
    std::vector<std::string> plugin_libs;
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

    const auto& nav_cfg = task_options_.navigate_to_pose();
    bool navigate_to_pose_listed = false;
    for (const auto& name : task_options_.navigators()) {
        if (name == "navigate_to_pose") {
            navigate_to_pose_listed = true;
            break;
        }
    }
    if (nav_cfg.enable() || navigate_to_pose_listed) {
        feedback.default_bt_xml_filename =
            nav_cfg.default_behavior_tree_file().empty()
                ? "navigate_to_pose.xml"
                : nav_cfg.default_behavior_tree_file();
        auto muxer_alias = std::shared_ptr<common::NavigatorMuxer>(
            &muxer_, [](common::NavigatorMuxer*) {});
        navigate_to_pose_ =
            std::make_shared<navigator::navigation::NavigateToPoseNavigator>(
                task_options_, task_context_, plugin_libs, feedback,
                muxer_alias, odom_smoother_);
    }
}

behavior_tree::BtStatus TaskScheduler::NavigateToPose(
    std::shared_ptr<const behavior_tree::proto::NavigateToPoseAction::Goal>
        goal) {
    if (!navigate_to_pose_) {
        AERROR << "NavigateToPose navigator is disabled in config.";
        return behavior_tree::BtStatus::FAILED;
    }
    cancel_requested_.store(false);
    return navigate_to_pose_->Bt().Run(
        goal, [this]() { return cancel_requested_.load(); });
}

void TaskScheduler::RequestCancel() {
    cancel_requested_.store(true);
    if (navigate_to_pose_) {
        navigate_to_pose_->Bt().RequestCancel();
    }
}

}  // namespace scheduler
}  // namespace tasks
}  // namespace autonomy
