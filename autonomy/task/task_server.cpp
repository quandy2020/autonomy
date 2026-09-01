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

#include <chrono>
#include <cmath>
#include <thread>

#include "autolink/autolink.hpp"
#include "autonomy/common/logging.hpp"
#include "autonomy/task/common/names.hpp"
#include "autonomy/task/behavior_tree/bt_defaults.hpp"
#include "autonomy/task/navigation/navigation_client.hpp"

namespace autonomy {
namespace task {

TaskServer::TaskServer() = default;
TaskServer::~TaskServer() { Shutdown(); }

proto::TaskServerOptions TaskServer::DefaultOptions() {
    proto::TaskServerOptions options;
    auto* scheduler = options.mutable_scheduler();
    scheduler->set_exclusive_navigation_tasks(true);
    scheduler->set_allow_preemption(true);
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

bool TaskServer::Configure(const proto::TaskServerOptions& options) {
    if (configured_) {
        Shutdown();
    }
    options_ = options;
    scheduler_ = std::make_shared<TaskScheduler>(options.scheduler());
    if (!scheduler_->Initialize(options)) {
        scheduler_.reset();
        return false;
    }

    node_ = autolink::CreateNode("task", "/autonomy/task");
    if (!node_) {
        scheduler_->Shutdown();
        scheduler_.reset();
        return false;
    }

    navigation_client_ = navigation::NavigationClient::Create(node_);
    if (!navigation_client_) {
        node_.reset();
        scheduler_->Shutdown();
        scheduler_.reset();
        return false;
    }
    navigation::NavigationClient::SetShared(navigation_client_);

    transform_listener_ = std::make_shared<TransformListener>();
    if (!transform_listener_->Start(node_)) {
        transform_listener_.reset();
    }

    AddApps(options.apps());
    Bind();
    configured_ = true;
    AINFO << "TaskServer configured";
    return true;
}

void TaskServer::Bind() {
    const auto period = std::chrono::milliseconds(
        options_.scheduler().feedback_period_ms() > 0
            ? options_.scheduler().feedback_period_ms()
            : 100);

    if (options_.apps().enable_navigation() && navigation_) {
        goal_pose_reader_ =
            node_->CreateReader<::automsgs::msgs::geometry_msgs::PoseStamped>(
                kGoalPose,
                [this](const std::shared_ptr<
                       ::automsgs::msgs::geometry_msgs::PoseStamped>& pose) {
                    if (!pose) {
                        return;
                    }
                    const auto now = std::chrono::steady_clock::now();
                    const double x = pose->pose().position().x();
                    const double y = pose->pose().position().y();
                    // Only drop near-duplicate Autoviz publishes (same click),
                    // never block intentional preemption to a new pose.
                    if (last_goal_pose_time_.time_since_epoch().count() > 0 &&
                        now - last_goal_pose_time_ <
                            std::chrono::milliseconds(120) &&
                        std::hypot(x - last_goal_pose_x_, y - last_goal_pose_y_) <
                            0.05) {
                        return;
                    }
                    last_goal_pose_time_ = now;
                    last_goal_pose_x_ = x;
                    last_goal_pose_y_ = y;
                    AINFO << "TaskServer: /goal_pose (" << x << ", " << y << ")";

                    {
                        std::lock_guard<std::mutex> lock(goal_pose_mutex_);
                        pending_goal_pose_ = *pose;
                    }
                    // Single worker applies the latest pose; rapid clicks coalesce.
                    if (goal_pose_worker_busy_.exchange(true)) {
                        return;
                    }
                    std::thread([this]() {
                        for (;;) {
                            std::optional<
                                ::automsgs::msgs::geometry_msgs::PoseStamped>
                                pose;
                            {
                                std::lock_guard<std::mutex> lock(
                                    goal_pose_mutex_);
                                pose = pending_goal_pose_;
                                pending_goal_pose_.reset();
                            }
                            if (!pose) {
                                goal_pose_worker_busy_.store(false);
                                // A click may have arrived after we cleared.
                                std::lock_guard<std::mutex> lock(
                                    goal_pose_mutex_);
                                if (pending_goal_pose_ &&
                                    !goal_pose_worker_busy_.exchange(true)) {
                                    continue;
                                }
                                return;
                            }
                            proto::NavigationGoal goal;
                            goal.set_command(proto::NAV_CMD_START);
                            goal.set_mode(proto::NAV_MODE_SINGLE_POSE);
                            *goal.add_goals() = *pose;
                            if (!Submit(goal)) {
                                AWARN << "TaskServer: /goal_pose submit failed "
                                         "(preempt/start)";
                            }
                        }
                    }).detach();
                });

        navigator_ = std::make_shared<common::Navigator>();
        if (!navigator_->Start(
                node_,
                [this](const proto::NavigationGoal& g) { return Submit(g); },
                [this](proto::NavigationFeedback* feedback) {
                    return navigation_ && navigation_->GetFeedback(feedback);
                },
                [this](proto::NavigationResult* result) {
                    return navigation_ && navigation_->GetResult(result);
                },
                period)) {
            navigator_.reset();
        }
    }

    BindDomain(
        options_.apps().enable_teleop(), teleop_, kTeleopGoal, kTeleopFeedback,
        period, &teleop_ingress_,
        [](const proto::TeleopFeedback& feedback) {
            return feedback.status() == proto::TELEOP_STATUS_TIMEOUT ||
                   feedback.status() == proto::TELEOP_STATUS_REJECTED ||
                   feedback.status() == proto::TELEOP_STATUS_IDLE;
        },
        []() {
            proto::TeleopFeedback feedback;
            feedback.set_status(proto::TELEOP_STATUS_REJECTED);
            return feedback;
        });

    BindDomain(
        options_.apps().enable_tracking(), tracking_, kTrackingGoal,
        kTrackingFeedback, period, &tracking_ingress_,
        [](const proto::TrackerFeedback& feedback) {
            return feedback.status() == proto::TRACKER_STATUS_SUCCEEDED ||
                   feedback.status() == proto::TRACKER_STATUS_FAILED ||
                   feedback.status() == proto::TRACKER_STATUS_CANCELED ||
                   feedback.status() == proto::TRACKER_STATUS_IDLE;
        },
        []() {
            proto::TrackerFeedback feedback;
            feedback.set_status(proto::TRACKER_STATUS_FAILED);
            return feedback;
        });

    BindDomain(
        options_.apps().enable_exploration(), exploration_, kExplorationGoal,
        kExplorationFeedback, period, &exploration_ingress_,
        [](const proto::ExplorationFeedback& feedback) {
            return feedback.status() == proto::EXPLORATION_STATUS_COMPLETED ||
                   feedback.status() == proto::EXPLORATION_STATUS_FAILED ||
                   feedback.status() == proto::EXPLORATION_STATUS_CANCELED ||
                   feedback.status() == proto::EXPLORATION_STATUS_IDLE;
        },
        []() {
            proto::ExplorationFeedback feedback;
            feedback.set_status(proto::EXPLORATION_STATUS_FAILED);
            return feedback;
        });

    BindDomain(
        options_.apps().enable_charging(), charging_, kChargingGoal,
        kChargingFeedback, period, &charging_ingress_,
        [](const proto::ChargingFeedback& feedback) {
            return feedback.status() == proto::DOCK_STATUS_SUCCEEDED ||
                   feedback.status() == proto::DOCK_STATUS_FAILED ||
                   feedback.status() == proto::DOCK_STATUS_CANCELED ||
                   feedback.status() == proto::DOCK_STATUS_IDLE;
        },
        []() {
            proto::ChargingFeedback feedback;
            feedback.set_status(proto::DOCK_STATUS_FAILED);
            return feedback;
        });

    BindDomain(
        options_.apps().enable_mapping(), mapping_, kMappingGoal,
        kMappingFeedback, period, &mapping_ingress_,
        [](const proto::MappingFeedback& feedback) {
            return feedback.status() == proto::MAP_STATUS_SUCCEEDED ||
                   feedback.status() == proto::MAP_STATUS_FAILED ||
                   feedback.status() == proto::MAP_STATUS_IDLE;
        },
        []() {
            proto::MappingFeedback feedback;
            feedback.set_status(proto::MAP_STATUS_FAILED);
            return feedback;
        });

    BindDomain(
        options_.apps().enable_localization(), localization_,
        kLocalizationGoal, kLocalizationFeedback, period,
        &localization_ingress_,
        [](const proto::LocalizationFeedback& feedback) {
            return feedback.status() == proto::LOCALIZATION_STATUS_SUCCEEDED ||
                   feedback.status() == proto::LOCALIZATION_STATUS_FAILED ||
                   feedback.status() == proto::LOCALIZATION_STATUS_CANCELED ||
                   feedback.status() == proto::LOCALIZATION_STATUS_IDLE;
        },
        []() {
            proto::LocalizationFeedback feedback;
            feedback.set_status(proto::LOCALIZATION_STATUS_FAILED);
            return feedback;
        });
}

void TaskServer::Unbind() {
    teleop_ingress_.Stop();
    tracking_ingress_.Stop();
    exploration_ingress_.Stop();
    charging_ingress_.Stop();
    mapping_ingress_.Stop();
    localization_ingress_.Stop();
    if (navigator_) {
        navigator_->Stop();
        navigator_.reset();
    }
    goal_pose_reader_.reset();
}

void TaskServer::AddApps(const proto::TaskAppOptions& apps) {
    RegisterBuiltinTasks(
        apps, [this](const auto& task) { Register(task); }, &navigation_,
        &tracking_, &teleop_, &exploration_, &charging_, &mapping_,
        &localization_);
}

bool TaskServer::Start() {
    return configured_ && scheduler_ && scheduler_->Start();
}

void TaskServer::Shutdown() {
    if (scheduler_) {
        scheduler_->Stop();
    }
    const auto stop = [](auto& t) {
        if (t) {
            t->Shutdown();
        }
    };
    stop(navigation_);
    stop(tracking_);
    stop(teleop_);
    stop(exploration_);
    stop(charging_);
    stop(mapping_);
    stop(localization_);
    if (scheduler_) {
        scheduler_->Shutdown();
        scheduler_.reset();
    }
    Unbind();
    navigation_.reset();
    tracking_.reset();
    teleop_.reset();
    exploration_.reset();
    charging_.reset();
    mapping_.reset();
    localization_.reset();
    navigation_client_.reset();
    if (transform_listener_) {
        transform_listener_->Stop();
        transform_listener_.reset();
    }
    node_.reset();
    configured_ = false;
}

bool TaskServer::IsRunning() const {
    return scheduler_ && scheduler_->IsRunning();
}

bool TaskServer::Submit(const proto::NavigationGoal& goal) {
    return Dispatch(navigation_, goal);
}
bool TaskServer::Submit(const proto::TrackerGoal& goal) {
    return Dispatch(tracking_, goal);
}
bool TaskServer::Submit(const proto::TeleopGoal& goal) {
    return Dispatch(teleop_, goal);
}
bool TaskServer::Submit(const proto::ExplorationGoal& goal) {
    return Dispatch(exploration_, goal);
}
bool TaskServer::Submit(const proto::ChargingGoal& goal) {
    return Dispatch(charging_, goal);
}
bool TaskServer::Submit(const proto::MappingGoal& goal) {
    return Dispatch(mapping_, goal);
}
bool TaskServer::Submit(const proto::LocalizationGoal& goal) {
    return Dispatch(localization_, goal);
}

std::vector<proto::ActiveTaskSnapshot> TaskServer::GetActiveSnapshots() const {
    return scheduler_ ? scheduler_->GetActiveSnapshots()
                      : std::vector<proto::ActiveTaskSnapshot>{};
}

}  // namespace task
}  // namespace autonomy
