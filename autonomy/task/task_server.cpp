/*
 * Copyright 2026 The Openbot Authors
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

#include "autonomy/task/task_server.hpp"

#include "autonomy/common/logging.hpp"
#include "autolink/autolink.hpp"
#include "autonomy/task/apps/behavior_tree/bt_defaults.hpp"
#include "autonomy/task/apps/navigation/navigation_client.hpp"

namespace autonomy {
namespace task {

TaskServer::TaskServer() = default;

TaskServer::~TaskServer() { Shutdown(); }

::autonomy::task::proto::TaskServerOptions TaskServer::DefaultOptions()
{
    ::autonomy::task::proto::TaskServerOptions options;
    auto* scheduler = options.mutable_scheduler();
    scheduler->set_exclusive_navigation_tasks(true);
    scheduler->set_feedback_period_ms(100);
    scheduler->set_default_task_timeout_sec(0.f);

    auto* apps = options.mutable_apps();
    apps->set_enable_navigation(true);
    apps->set_enable_tracking(true);
    apps->set_enable_teleop(true);
    apps->set_enable_exploration(true);
    apps->set_enable_charging(true);
    apps->set_enable_mapping(true);
    apps->set_enable_localization(true);

    BtDefaults::Apply(&options);
    return options;
}

bool TaskServer::Configure(
    const ::autonomy::task::proto::TaskServerOptions& options)
{
    if (configured_) {
        Shutdown();
    }

    options_ = options;
    scheduler_ = std::make_shared<TaskScheduler>(options.scheduler());
    if (!scheduler_->Initialize(options)) {
        AERROR << "TaskServer scheduler initialize failed";
        scheduler_.reset();
        return false;
    }

    task_node_ = autolink::CreateNode("task", "/autonomy/task");
    if (!task_node_) {
        AERROR << "TaskServer: failed to create autolink node";
        scheduler_->Shutdown();
        scheduler_.reset();
        return false;
    }

    navigation_client_ = navigation::NavigationClient::Create(task_node_);
    if (!navigation_client_) {
        AERROR << "TaskServer: failed to create navigation client";
        task_node_.reset();
        scheduler_->Shutdown();
        scheduler_.reset();
        return false;
    }
    navigation::NavigationClient::SetShared(navigation_client_);

    autolink_tf_listener_ = std::make_shared<AutolinkTfListener>();
    if (!autolink_tf_listener_->Start(task_node_)) {
        AWARN << "TaskServer: Autolink TF listener failed (costmap/assist need "
                 "/tf or tf)";
        autolink_tf_listener_.reset();
    }

    RegisterEnabledPlugins(options.apps());
    if (options.apps().enable_teleop() && teleop_) {
        const auto feedback_period = std::chrono::milliseconds(
            options.scheduler().feedback_period_ms() > 0
                ? options.scheduler().feedback_period_ms()
                : 100);

        teleop_feedback_publisher_ =
            std::make_shared<TeleopFeedbackPublisher>();
        if (!teleop_feedback_publisher_->Start(task_node_, teleop_,
                                               feedback_period)) {
            AWARN << "TaskServer: teleop feedback publisher failed";
            teleop_feedback_publisher_.reset();
        }

        const auto feedback_publisher = teleop_feedback_publisher_;
        teleop_goal_ingress_ = std::make_shared<TeleopGoalIngress>();
        if (!teleop_goal_ingress_->Start(
                task_node_,
                [this](const ::autonomy::task::proto::TeleopGoal& g) {
                    return SubmitTeleopGoal(g);
                },
                [feedback_publisher](
                    const ::autonomy::task::proto::TeleopGoal& /*goal*/) {
                    if (!feedback_publisher) {
                        return;
                    }
                    ::autonomy::task::proto::TeleopFeedback feedback;
                    feedback.set_status(
                        ::autonomy::task::proto::TELEOP_STATUS_REJECTED);
                    feedback_publisher->Publish(feedback);
                })) {
            AWARN << "TaskServer: teleop goal ingress failed; "
                     "SubmitTeleopGoal in-process only";
            teleop_goal_ingress_.reset();
        }
    }
    configured_ = true;
    AINFO << "TaskServer configured";
    return true;
}

void TaskServer::RegisterEnabledPlugins(
    const ::autonomy::task::proto::TaskAppOptions& apps)
{
    const auto wire = [this](const auto& task) {
        if (task) {
            task->SetAutolinkNode(task_node_);
            task->SetNavigationClient(navigation_client_);
            scheduler_->RegisterTask(task);
        }
    };

    if (apps.enable_navigation()) {
        navigation_ = std::make_shared<NavigationTask>();
        wire(navigation_);
    }
    if (apps.enable_tracking()) {
        tracking_ = std::make_shared<TrackerTask>();
        wire(tracking_);
    }
    if (apps.enable_teleop()) {
        teleop_ = std::make_shared<TeleopTask>();
        wire(teleop_);
    }
    if (apps.enable_exploration()) {
        AWARN << "TaskServer: exploration app temporarily disabled "
                 "(automsgs migration)";
    }
    if (apps.enable_charging()) {
        charging_ = std::make_shared<ChargingTask>();
        wire(charging_);
    }
    if (apps.enable_mapping()) {
        mapping_ = std::make_shared<MappingTask>();
        wire(mapping_);
    }
    if (apps.enable_localization()) {
        localization_ = std::make_shared<LocalizationTask>();
        wire(localization_);
    }
}

bool TaskServer::Start()
{
    if (!configured_ || !scheduler_) {
        AERROR << "TaskServer::Start before Configure";
        return false;
    }
    return scheduler_->Start();
}

void TaskServer::Shutdown()
{
    if (scheduler_) {
        scheduler_->Stop();
    }

    const auto shutdown_plugin = [](auto& task) {
        if (task) {
            task->Shutdown();
        }
    };
    shutdown_plugin(navigation_);
    shutdown_plugin(tracking_);
    shutdown_plugin(teleop_);
    shutdown_plugin(charging_);
    shutdown_plugin(mapping_);
    shutdown_plugin(localization_);

    if (scheduler_) {
        scheduler_->Shutdown();
        scheduler_.reset();
    }
    navigation_.reset();
    tracking_.reset();
    teleop_.reset();
    charging_.reset();
    mapping_.reset();
    localization_.reset();
    navigation_client_.reset();
    if (teleop_goal_ingress_) {
        teleop_goal_ingress_->Stop();
        teleop_goal_ingress_.reset();
    }
    if (teleop_feedback_publisher_) {
        teleop_feedback_publisher_->Stop();
        teleop_feedback_publisher_.reset();
    }
    if (autolink_tf_listener_) {
        autolink_tf_listener_->Stop();
        autolink_tf_listener_.reset();
    }
    task_node_.reset();
    configured_ = false;
}

bool TaskServer::IsRunning() const
{
    return scheduler_ && scheduler_->IsRunning();
}

bool TaskServer::SubmitNavigationGoal(
    const ::autonomy::task::proto::NavigationGoal& goal)
{
    return SubmitGoal(navigation_, goal);
}

bool TaskServer::SubmitTrackerGoal(
    const ::autonomy::task::proto::TrackerGoal& goal)
{
    return SubmitGoal(tracking_, goal);
}

bool TaskServer::SubmitTeleopGoal(
    const ::autonomy::task::proto::TeleopGoal& goal)
{
    return SubmitGoal(teleop_, goal);
}

bool TaskServer::SubmitExplorationGoal(
    const ::autonomy::task::proto::ExplorationGoal& goal)
{
    (void)goal;
    AWARN << "TaskServer::SubmitExplorationGoal: exploration temporarily disabled";
    return false;
}

bool TaskServer::SubmitChargingGoal(
    const ::autonomy::task::proto::ChargingGoal& goal)
{
    return SubmitGoal(charging_, goal);
}

bool TaskServer::SubmitMappingGoal(
    const ::autonomy::task::proto::MappingGoal& goal)
{
    return SubmitGoal(mapping_, goal);
}

bool TaskServer::SubmitLocalizationGoal(
    const ::autonomy::task::proto::LocalizationGoal& goal)
{
    return SubmitGoal(localization_, goal);
}

std::vector<::autonomy::task::proto::ActiveTaskSnapshot>
TaskServer::GetActiveSnapshots() const
{
    if (!scheduler_) {
        return {};
    }
    return scheduler_->GetActiveSnapshots();
}

}  // namespace task
}  // namespace autonomy
