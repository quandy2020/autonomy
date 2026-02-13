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

#include "autonomy/control/controller_server.hpp"

#include "autolink/autolink/common/log.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/std_msgs.hpp"
#include "autonomy/control/common/controller_exceptions.hpp"
#include "autonomy/control/utils/conversions.hpp"
#include "autonomy/map/costmap_2d/utils/geometry_utils.hpp"

namespace autonomy {
namespace control {

using Time = commsgs::builtin_interfaces::Time;
using namespace autonomy::control::utils;

ControllerServer::ControllerServer(const proto::ControllerOptions& options) : options_{options} {
    progress_checker_loader_ =
        std::make_shared<::autolink::class_loader::ClassLoader>("libautonomy_control_plugins.so");
    default_progress_checker_ids_ = {"progress_checker"};
    default_progress_checker_types_ = {"nav2_controller::SimpleProgressChecker"};
    goal_checker_loader_ = std::make_shared<::autolink::class_loader::ClassLoader>("libautonomy_control_plugins.so");
    default_goal_checker_ids_ = {"goal_checker"};
    default_goal_checker_types_ = {"nav2_controller::SimpleGoalChecker"};
    controller_loader_ = std::make_shared<::autolink::class_loader::ClassLoader>("libautonomy_control_plugins.so");
    default_ids_ = {"FollowPath"};
    default_types_ = {"autonomy::control::DWBLocalPlanner"};
    costmap_update_timeout_ = commsgs::builtin_interfaces::Duration::FromSeconds(300.0);

    AINFO << "Control server init successfully.";
}

ControllerServer::~ControllerServer() {
    Shutdown();
    AINFO << "Control server shutdown successfully.";
}

void ControllerServer::Start() {
    AINFO << "Starting controller server...";

    // 1. 读取和初始化参数
    controller_frequency_ = 20.0;  // 默认值，可以从 options_ 中读取
    min_x_velocity_threshold_ = 0.0001;
    min_y_velocity_threshold_ = 0.0001;
    min_theta_velocity_threshold_ = 0.0001;
    failure_tolerance_ = 0.0;
    use_realtime_priority_ = false;
    publish_zero_velocity_ = true;
    costmap_update_timeout_ = commsgs::builtin_interfaces::Duration::FromSeconds(0.3);  // 300ms
    std::string speed_limit_topic = "speed_limit";

    // 2. 初始化成本地图
    if (!costmap_wrapper_) {
        costmap_wrapper_ = std::make_shared<map::costmap_2d::Costmap2DWrapper>(options_.costmap_2d_options(),
                                                                               "local_costmap", node_.get());
    }
    costmap_wrapper_->Start();

    // 启动成本地图线程
    costmap_thread_ = std::make_unique<std::thread>([this]() {
        // 成本地图在独立线程中运行
        // 这里可以添加成本地图的更新循环
    });

    // 3. 加载插件
    // 3.1 Progress Checker 插件
    progress_checker_ids_ = default_progress_checker_ids_;
    progress_checker_types_ = default_progress_checker_types_;
    progress_checker_types_.resize(progress_checker_ids_.size());

    for (size_t i = 0; i != progress_checker_ids_.size(); i++) {
        try {
            progress_checker_types_[i] = default_progress_checker_types_[i];
            common::ProgressChecker::SharedPtr progress_checker =
                progress_checker_loader_->CreateClassObj<common::ProgressChecker>(progress_checker_types_[i]);
            if (!progress_checker) {
                throw std::runtime_error("Failed to create progress checker: " + progress_checker_types_[i]);
            }
            AINFO << "Created progress_checker: " << progress_checker_ids_[i] << " of type "
                  << progress_checker_types_[i];
            progress_checker->Initialize(progress_checker_ids_[i]);
            progress_checkers_.insert({progress_checker_ids_[i], progress_checker});
        } catch (const std::exception& ex) {
            AFATAL << "Failed to create progress_checker. Exception: " << ex.what();
            throw;
        }
    }

    for (size_t i = 0; i != progress_checker_ids_.size(); i++) {
        progress_checker_ids_concat_ += progress_checker_ids_[i] + std::string(" ");
    }
    AINFO << "Controller Server has " << progress_checker_ids_concat_ << "progress checkers available.";

    // 3.2 Goal Checker 插件
    goal_checker_ids_ = default_goal_checker_ids_;
    goal_checker_types_ = default_goal_checker_types_;
    goal_checker_types_.resize(goal_checker_ids_.size());

    for (size_t i = 0; i != goal_checker_ids_.size(); i++) {
        try {
            goal_checker_types_[i] = default_goal_checker_types_[i];
            common::GoalChecker::SharedPtr goal_checker =
                goal_checker_loader_->CreateClassObj<common::GoalChecker>(goal_checker_types_[i]);
            if (!goal_checker) {
                throw std::runtime_error("Failed to create goal checker: " + goal_checker_types_[i]);
            }
            AINFO << "Created goal checker: " << goal_checker_ids_[i] << " of type " << goal_checker_types_[i];
            goal_checker->Initialize(goal_checker_ids_[i], costmap_wrapper_);
            goal_checkers_.insert({goal_checker_ids_[i], goal_checker});
        } catch (const std::exception& ex) {
            AFATAL << "Failed to create goal checker. Exception: " << ex.what();
            throw;
        }
    }

    for (size_t i = 0; i != goal_checker_ids_.size(); i++) {
        goal_checker_ids_concat_ += goal_checker_ids_[i] + std::string(" ");
    }
    AINFO << "Controller Server has " << goal_checker_ids_concat_ << "goal checkers available.";

    // 3.3 Controller 插件
    controller_ids_ = default_ids_;
    controller_types_ = default_types_;
    controller_types_.resize(controller_ids_.size());

    for (size_t i = 0; i != controller_ids_.size(); i++) {
        try {
            controller_types_[i] = default_types_[i];
            common::ControllerInterface::SharedPtr controller =
                controller_loader_->CreateClassObj<common::ControllerInterface>(controller_types_[i]);
            if (!controller) {
                throw std::runtime_error("Failed to create controller: " + controller_types_[i]);
            }
            AINFO << "Created controller: " << controller_ids_[i] << " of type " << controller_types_[i];
            // 需要 TF buffer，暂时传 nullptr，后续需要从 costmap_wrapper_ 获取
            controller->Configure(options_, controller_ids_[i], nullptr, costmap_wrapper_);
            controllers_.insert({controller_ids_[i], controller});
        } catch (const std::exception& ex) {
            AFATAL << "Failed to create controller. Exception: " << ex.what();
            throw;
        }
    }

    for (size_t i = 0; i != controller_ids_.size(); i++) {
        controller_ids_concat_ += controller_ids_[i] + std::string(" ");
    }
    AINFO << "Controller Server has " << controller_ids_concat_ << "controllers available.";

    // 4. 创建发布者和订阅者
    // 4.1 Odometry 订阅者
    odom_sub_ = std::make_unique<utils::OdomSmoother>(node_, 0.3, "odom");

    // 4.2 速度发布者
    vel_publisher_ = node_->CreateWriter<commsgs::geometry_msgs::TwistStamped>("cmd_vel");

    // 4.3 速度限制订阅者
    speed_limit_subscriber_ = node_->CreateReader<commsgs::planning_msgs::SpeedLimit>(
        speed_limit_topic, std::bind(&ControllerServer::SpeedLimitCallback, this, std::placeholders::_1));

    // 5. 创建 Action Server
    try {
        action_server_ =
            std::make_shared<ActionServer>(node_, "follow_path", std::bind(&ControllerServer::ComputeControl, this),
                                           nullptr, std::chrono::milliseconds(500), use_realtime_priority_);
    } catch (const std::runtime_error& e) {
        AERROR << "Error creating action server! " << e.what();
        throw;
    }

    // 6. 激活插件
    for (auto it = controllers_.begin(); it != controllers_.end(); ++it) {
        it->second->Activate();
    }

    // 激活 Action Server
    action_server_->Activate();

    AINFO << "Controller server started successfully.";
}

void ControllerServer::Shutdown() {
    AINFO << "Shutting down controller server...";

    // 停用 Action Server
    if (action_server_) {
        action_server_->Deactivate();
        action_server_.reset();
    }

    // 停用所有 Controller 插件
    for (auto it = controllers_.begin(); it != controllers_.end(); ++it) {
        it->second->Deactivate();
    }

    // 停用成本地图
    if (costmap_wrapper_) {
        costmap_wrapper_->Stop();
    }

    // 发布零速度
    if (publish_zero_velocity_) {
        PublishZeroVelocity();
    }

    // 清理成本地图线程
    if (costmap_thread_ && costmap_thread_->joinable()) {
        costmap_thread_->join();
        costmap_thread_.reset();
    }

    // 清理插件
    for (auto it = controllers_.begin(); it != controllers_.end(); ++it) {
        it->second->Cleanup();
    }
    controllers_.clear();
    goal_checkers_.clear();
    progress_checkers_.clear();

    // 清理加载器
    controller_loader_.reset();
    goal_checker_loader_.reset();
    progress_checker_loader_.reset();

    // 清理发布者和订阅者
    odom_sub_.reset();
    vel_publisher_.reset();
    speed_limit_subscriber_.reset();

    AINFO << "Control server shutdown successfully.";
}

bool ControllerServer::FindControllerId(const std::string& c_name, std::string& current_controller) {
    if (controllers_.find(c_name) == controllers_.end()) {
        if (controllers_.size() == 1 && c_name.empty()) {
            LOG_FIRST_N(WARNING, 1) << "No controller was specified in action call."
                                    << " Server will use only plugin loaded " << controller_ids_concat_
                                    << ". This warning will appear once.";
        } else {
            AERROR << "FollowPath called with controller name " << c_name
                   << ", which does not exist. Available controllers are: " << controller_ids_concat_;
            return false;
        }
    } else {
        ADEBUG << " Selected controller: " << c_name;
        current_controller = c_name;
    }

    return true;
}

bool ControllerServer::FindGoalCheckerId(const std::string& c_name, std::string& current_goal_checker) {
    if (goal_checkers_.find(c_name) == goal_checkers_.end()) {
        if (goal_checkers_.size() == 1 && c_name.empty()) {
            AWARN << "No goal checker was specified in parameter "
                  << "'current_goal_checker'."
                  << " Server will use only plugin loaded " << goal_checker_ids_concat_
                  << ". This warning will appear once.";
            current_goal_checker = goal_checkers_.begin()->first;
        } else {
            AERROR << "FollowPath called with goal_checker name " << c_name
                   << " in parameter 'current_goal_checker', which does not "
                      "exist. Available goal checkers are: "
                   << goal_checker_ids_concat_;
            return false;
        }
    } else {
        ADEBUG << "Selected goal checker: " << c_name;
        current_goal_checker = c_name;
    }

    return true;
}

bool ControllerServer::FindProgressCheckerId(const std::string& c_name, std::string& current_progress_checker) {
    if (progress_checkers_.find(c_name) == progress_checkers_.end()) {
        if (progress_checkers_.size() == 1 && c_name.empty()) {
            AWARN << "No progress checker was specified in parameter "
                  << "'current_progress_checker'."
                  << " Server will use only plugin loaded " << progress_checker_ids_concat_
                  << ". This warning will appear once.";
            current_progress_checker = progress_checkers_.begin()->first;
        } else {
            AERROR << "FollowPath called with progress_checker name " << c_name
                   << " in parameter 'current_progress_checker', which does not "
                      "exist. Available progress checkers are: "
                   << progress_checker_ids_concat_;
            return false;
        }
    } else {
        ADEBUG << "Selected progress checker: " << c_name;
        current_progress_checker = c_name;
    }

    return true;
}

void ControllerServer::ComputeControl() {
    AINFO << "Received a goal, begin computing control effort.";

    try {
        // Get goal from current goal handle
        if (!action_server_) {
            return;  // No active goal
        }
        auto goal = action_server_->GetCurrentGoal();
        if (!goal) {
            return;  //  goal would be nullptr if action_server_ is inactivate.
        }

        std::string c_name = goal->controller_id();
        std::string current_controller;
        if (FindControllerId(c_name, current_controller)) {
            current_controller_ = current_controller;
        } else {
            throw common::InvalidController("Failed to find controller name: " + c_name);
        }

        std::string gc_name = goal->goal_checker_id();
        std::string current_goal_checker;
        if (FindGoalCheckerId(gc_name, current_goal_checker)) {
            current_goal_checker_ = current_goal_checker;
        } else {
            throw common::ControllerException("Failed to find goal checker name: " + gc_name);
        }

        std::string pc_name = goal->progress_checker_id();
        std::string current_progress_checker;
        if (FindProgressCheckerId(pc_name, current_progress_checker)) {
            current_progress_checker_ = current_progress_checker;
        } else {
            throw common::ControllerException("Failed to find progress checker name: " + pc_name);
        }

        // Convert proto Path to commsgs Path
        commsgs::planning_msgs::Path path;
        path.header = commsgs::std_msgs::FromProto(goal->path().header());
        path.poses.reserve(goal->path().poses_size());
        for (const auto& pose_proto : goal->path().poses()) {
            path.poses.push_back(commsgs::geometry_msgs::FromProto(pose_proto));
        }
        SetPlannerPath(path);
        progress_checkers_[current_progress_checker_]->Reset();

        last_valid_cmd_time_ = Time::Now();
        ::autolink::Rate loop_rate(controller_frequency_);
        while (::autolink::OK()) {
            auto start_time = Time::Now();

            if (action_server_ == nullptr || !action_server_->GetCurrentGoal()) {
                AINFO << "Action server unavailable or no active goal. Stopping.";
                return;
            }

            if (action_server_->IsCancelRequested()) {
                // 检查控制器是否支持取消
                if (controllers_[current_controller_]) {
                    // 如果控制器支持取消，等待其完成
                    // 注意：ControllerInterface 没有 Cancel() 方法，这里先直接终止
                    AINFO << "Cancellation requested. Stopping the robot.";
                    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
                    result->set_error_code(
                        autonomy::tasks::behavior_tree::proto::FollowPathErrorCode::FOLLOW_PATH_ERROR_NONE);
                    action_server_->TerminateAll(result);
                    OnGoalExit();
                    return;
                } else {
                    AINFO << "Cancellation requested. Stopping the robot.";
                    std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
                    result->set_error_code(
                        autonomy::tasks::behavior_tree::proto::FollowPathErrorCode::FOLLOW_PATH_ERROR_NONE);
                    action_server_->TerminateCurrent(result);
                    OnGoalExit();
                    return;
                }
            }
            // Don't compute a trajectory until costmap is valid (after clear
            // costmap)
            ::autolink::Rate r(100.0);
            auto waiting_start = Time::Now();
            while (!costmap_wrapper_->isCurrent()) {
                auto elapsed = Time::Now() - waiting_start;
                if (elapsed.Seconds() > costmap_update_timeout_.Seconds()) {
                    throw common::ControllerTimedOut("Costmap timed out waiting for update");
                }
                r.Sleep();
            }

            UpdateGlobalPath();

            ComputeAndPublishVelocity();

            if (IsGoalReached()) {
                AINFO << "Reached the goal!";
                std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
                result->set_error_code(
                    autonomy::tasks::behavior_tree::proto::FollowPathErrorCode::FOLLOW_PATH_ERROR_NONE);
                action_server_->SucceededCurrent(result);
                break;
            }

            auto cycle_duration = Time::Now() - start_time;
            double cycle_seconds = cycle_duration.Seconds();
            if (cycle_seconds > 0) {
                loop_rate.Sleep();
                if (cycle_seconds > 1.0 / controller_frequency_) {
                    AWARN << "Control loop missed its desired rate of " << controller_frequency_
                          << " Hz. Current loop rate is " << 1.0 / cycle_seconds << " Hz.";
                }
            }
        }
    } catch (common::InvalidController& e) {
        AERROR << e.what();
        OnGoalExit();
        std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
        result->set_error_code(
            autonomy::tasks::behavior_tree::proto::FollowPathErrorCode::FOLLOW_PATH_ERROR_INVALID_CONTROLLER);
        if (action_server_) {
            action_server_->TerminateCurrent(result);
        }
        return;
    } catch (common::ControllerTFError& e) {
        AERROR << e.what();
        OnGoalExit();
        std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
        result->set_error_code(autonomy::tasks::behavior_tree::proto::FollowPathErrorCode::FOLLOW_PATH_ERROR_TF_ERROR);
        if (action_server_) {
            action_server_->TerminateCurrent(result);
        }
        return;
    } catch (common::NoValidControl& e) {
        AERROR << e.what();
        OnGoalExit();
        std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
        result->set_error_code(
            autonomy::tasks::behavior_tree::proto::FollowPathErrorCode::FOLLOW_PATH_ERROR_NO_VALID_CONTROL);
        if (action_server_) {
            action_server_->TerminateCurrent(result);
        }
        return;
    } catch (common::FailedToMakeProgress& e) {
        AERROR << e.what();
        OnGoalExit();
        std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
        result->set_error_code(
            autonomy::tasks::behavior_tree::proto::FollowPathErrorCode::FOLLOW_PATH_ERROR_FAILED_TO_MAKE_PROGRESS);
        if (action_server_) {
            action_server_->TerminateCurrent(result);
        }
        return;
    } catch (common::PatienceExceeded& e) {
        AERROR << e.what();
        OnGoalExit();
        std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
        result->set_error_code(
            autonomy::tasks::behavior_tree::proto::FollowPathErrorCode::FOLLOW_PATH_ERROR_PATIENCE_EXCEEDED);
        if (action_server_) {
            action_server_->TerminateCurrent(result);
        }
        return;
    } catch (common::InvalidPath& e) {
        AERROR << e.what();
        OnGoalExit();
        std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
        result->set_error_code(
            autonomy::tasks::behavior_tree::proto::FollowPathErrorCode::FOLLOW_PATH_ERROR_INVALID_PATH);
        if (action_server_) {
            action_server_->TerminateCurrent(result);
        }
        return;
    } catch (common::ControllerTimedOut& e) {
        AERROR << e.what();
        OnGoalExit();
        std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
        result->set_error_code(
            autonomy::tasks::behavior_tree::proto::FollowPathErrorCode::FOLLOW_PATH_ERROR_CONTROLLER_TIMED_OUT);
        if (action_server_) {
            action_server_->TerminateCurrent(result);
        }
        return;
    } catch (common::ControllerException& e) {
        AERROR << e.what();
        OnGoalExit();
        std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
        result->set_error_code(autonomy::tasks::behavior_tree::proto::FollowPathErrorCode::FOLLOW_PATH_ERROR_UNKNOWN);
        if (action_server_) {
            action_server_->TerminateCurrent(result);
        }
        return;
    } catch (std::exception& e) {
        AERROR << e.what();
        OnGoalExit();
        std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
        result->set_error_code(autonomy::tasks::behavior_tree::proto::FollowPathErrorCode::FOLLOW_PATH_ERROR_UNKNOWN);
        if (action_server_) {
            action_server_->TerminateCurrent(result);
        }
        return;
    }

    ADEBUG << "Controller succeeded, setting result";
    OnGoalExit();
    if (action_server_) {
        std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
        result->set_error_code(autonomy::tasks::behavior_tree::proto::FollowPathErrorCode::FOLLOW_PATH_ERROR_NONE);
        action_server_->SucceededCurrent(result);
    }
}

void ControllerServer::SetPlannerPath(const commsgs::planning_msgs::Path& path) {
    ADEBUG << "Providing path to the controller " << current_controller_;
    if (path.poses.empty()) {
        throw common::InvalidPath("Path is empty.");
    }
    controllers_[current_controller_]->SetPlan(path);

    end_pose_ = path.poses.back();
    end_pose_.header.frame_id = path.header.frame_id;
    goal_checkers_[current_goal_checker_]->Reset();

    ADEBUG << "Path end point is (" << end_pose_.pose.position.x << ", " << end_pose_.pose.position.y << ")";
    current_path_ = path;
}

void ControllerServer::ComputeAndPublishVelocity() {
    commsgs::geometry_msgs::PoseStamped pose;

    if (!GetRobotPose(pose)) {
        throw common::ControllerTFError("Failed to obtain robot pose");
    }

    if (!progress_checkers_[current_progress_checker_]->Check(pose)) {
        throw common::FailedToMakeProgress("Failed to make progress");
    }

    // Get odometry twist from odom subscriber
    commsgs::geometry_msgs::Twist twist_3d_raw = odom_sub_->getTwist();
    commsgs::planning_msgs::Twist2D twist_plan_raw = twist3Dto2D(twist_3d_raw);
    commsgs::geometry_msgs::Twist2D twist_geo;
    twist_geo.x = twist_plan_raw.x;
    twist_geo.y = twist_plan_raw.y;
    twist_geo.theta = twist_plan_raw.theta;
    commsgs::geometry_msgs::Twist2D twist = GetThresholdedTwist(twist_geo);

    // Convert geometry_msgs::Twist2D to planning_msgs::Twist2D for twist2Dto3D
    commsgs::planning_msgs::Twist2D twist_plan;
    twist_plan.x = twist.x;
    twist_plan.y = twist.y;
    twist_plan.theta = twist.theta;

    // Convert to TwistStamped for ComputeVelocityCommands
    commsgs::geometry_msgs::Twist twist_3d = twist2Dto3D(twist_plan);
    commsgs::geometry_msgs::TwistStamped twist_stamped;
    twist_stamped.twist = twist_3d;
    twist_stamped.header.frame_id = costmap_wrapper_->getBaseFrameID();
    twist_stamped.header.stamp = Time::Now();

    commsgs::geometry_msgs::TwistStamped cmd_vel_2d;
    std::string message;

    try {
        controllers_[current_controller_]->ComputeVelocityCommands(
            pose, twist_stamped, cmd_vel_2d, goal_checkers_[current_goal_checker_].get(), message);
        last_valid_cmd_time_ = Time::Now();
        cmd_vel_2d.header.frame_id = costmap_wrapper_->getBaseFrameID();
        cmd_vel_2d.header.stamp = last_valid_cmd_time_;
        // Only no valid control exception types are valid to attempt to have control patience, as
        // other types will not be resolved with more attempts
    } catch (common::NoValidControl& e) {
        if (failure_tolerance_ > 0 || failure_tolerance_ == -1.0) {
            AWARN << e.what();
            cmd_vel_2d.twist.angular.x = 0;
            cmd_vel_2d.twist.angular.y = 0;
            cmd_vel_2d.twist.angular.z = 0;
            cmd_vel_2d.twist.linear.x = 0;
            cmd_vel_2d.twist.linear.y = 0;
            cmd_vel_2d.twist.linear.z = 0;
            cmd_vel_2d.header.frame_id = costmap_wrapper_->getBaseFrameID();
            cmd_vel_2d.header.stamp = Time::Now();
            if ((Time::Now() - last_valid_cmd_time_).Seconds() > failure_tolerance_ && failure_tolerance_ != -1.0) {
                throw common::PatienceExceeded("Controller patience exceeded");
            }
        } else {
            throw common::NoValidControl(e.what());
        }
    }

    std::shared_ptr<Action::Feedback> feedback = std::make_shared<Action::Feedback>();
    feedback->set_speed(std::hypot(cmd_vel_2d.twist.linear.x, cmd_vel_2d.twist.linear.y));

    // Find the closest pose to current pose on global path
    commsgs::planning_msgs::Path& current_path = current_path_;
    auto find_closest_pose_idx = [&pose, &current_path]() {
        size_t closest_pose_idx = 0;
        double curr_min_dist = std::numeric_limits<double>::max();
        for (size_t curr_idx = 0; curr_idx < current_path.poses.size(); ++curr_idx) {
            double curr_dist = map::costmap_2d::utils::euclidean_distance(pose, current_path.poses[curr_idx]);
            if (curr_dist < curr_min_dist) {
                curr_min_dist = curr_dist;
                closest_pose_idx = curr_idx;
            }
        }
        return closest_pose_idx;
    };

    feedback->set_distance_to_goal(
        map::costmap_2d::utils::calculate_path_length(current_path_, find_closest_pose_idx()));
    // Publish feedback using current goal handle if available
    if (action_server_) {
        action_server_->PublishFeedback(feedback);
    }

    ADEBUG << "Publishing velocity at time " << Time::Now().Seconds();
    PublishVelocity(cmd_vel_2d);
}

void ControllerServer::UpdateGlobalPath() {
    // Check if we have a preemption request (new goal was accepted)
    if (action_server_ && action_server_->IsPreemptRequested()) {
        auto goal = action_server_->AcceptPendingGoal();
        if (!goal) {
            return;
        }

        AINFO << "Passing new path to controller.";

        std::string current_controller;
        if (FindControllerId(goal->controller_id(), current_controller)) {
            current_controller_ = current_controller;
        } else {
            AINFO << "Terminating action, invalid controller " << goal->controller_id() << " requested.";
            std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
            result->set_error_code(
                autonomy::tasks::behavior_tree::proto::FollowPathErrorCode::FOLLOW_PATH_ERROR_UNKNOWN);
            if (action_server_) {
                action_server_->TerminateCurrent(result);
            }
            return;
        }

        std::string current_goal_checker;
        if (FindGoalCheckerId(goal->goal_checker_id(), current_goal_checker)) {
            current_goal_checker_ = current_goal_checker;
        } else {
            AINFO << "Terminating action, invalid goal checker " << goal->goal_checker_id() << " requested.";
            std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
            result->set_error_code(
                autonomy::tasks::behavior_tree::proto::FollowPathErrorCode::FOLLOW_PATH_ERROR_UNKNOWN);
            if (action_server_) {
                action_server_->TerminateCurrent(result);
            }
            return;
        }

        std::string current_progress_checker;
        if (FindProgressCheckerId(goal->progress_checker_id(), current_progress_checker)) {
            if (current_progress_checker_ != current_progress_checker) {
                AINFO << "Change of progress checker " << current_progress_checker << " requested, resetting it";
                current_progress_checker_ = current_progress_checker;
                progress_checkers_[current_progress_checker_]->Reset();
            }
        } else {
            AINFO << "Terminating action, invalid progress checker " << goal->progress_checker_id() << " requested.";
            auto result = std::make_shared<Action::Result>();
            result->set_error_code(
                autonomy::tasks::behavior_tree::proto::FollowPathErrorCode::FOLLOW_PATH_ERROR_UNKNOWN);
            if (action_server_) {
                action_server_->TerminateCurrent(result);
            }
            return;
        }

        // Convert proto Path to commsgs Path
        commsgs::planning_msgs::Path path;
        path.header = commsgs::std_msgs::FromProto(goal->path().header());
        path.poses.reserve(goal->path().poses_size());
        for (const auto& pose_proto : goal->path().poses()) {
            path.poses.push_back(commsgs::geometry_msgs::FromProto(pose_proto));
        }
        SetPlannerPath(path);
    }
}

void ControllerServer::PublishVelocity(const commsgs::geometry_msgs::TwistStamped& velocity) {
    // 验证速度消息（检查 NaN/Inf）
    if (std::isnan(velocity.twist.linear.x) || std::isinf(velocity.twist.linear.x) ||
        std::isnan(velocity.twist.linear.y) || std::isinf(velocity.twist.linear.y) ||
        std::isnan(velocity.twist.linear.z) || std::isinf(velocity.twist.linear.z) ||
        std::isnan(velocity.twist.angular.x) || std::isinf(velocity.twist.angular.x) ||
        std::isnan(velocity.twist.angular.y) || std::isinf(velocity.twist.angular.y) ||
        std::isnan(velocity.twist.angular.z) || std::isinf(velocity.twist.angular.z)) {
        AERROR << "Velocity message contains NaNs or Infs! Ignoring as invalid!";
        return;
    }

    if (vel_publisher_) {
        vel_publisher_->Write(velocity);
    }
}

void ControllerServer::PublishZeroVelocity() {
    commsgs::geometry_msgs::TwistStamped velocity;
    velocity.twist.angular.x = 0;
    velocity.twist.angular.y = 0;
    velocity.twist.angular.z = 0;
    velocity.twist.linear.x = 0;
    velocity.twist.linear.y = 0;
    velocity.twist.linear.z = 0;
    velocity.header.frame_id = costmap_wrapper_->getBaseFrameID();
    velocity.header.stamp = Time::Now();
    PublishVelocity(velocity);
}

void ControllerServer::OnGoalExit() {
    if (publish_zero_velocity_) {
        PublishZeroVelocity();
    }

    // Reset the state of the controllers after the task has ended
    for (auto it = controllers_.begin(); it != controllers_.end(); ++it) {
        it->second->Reset();
    }
}

bool ControllerServer::IsGoalReached() {
    commsgs::geometry_msgs::PoseStamped pose;

    if (!GetRobotPose(pose)) {
        return false;
    }

    // Get odometry twist from odom subscriber
    commsgs::geometry_msgs::Twist twist_3d_raw = odom_sub_->getTwist();
    commsgs::planning_msgs::Twist2D twist_plan_raw = twist3Dto2D(twist_3d_raw);
    commsgs::geometry_msgs::Twist2D twist_geo_raw;
    twist_geo_raw.x = twist_plan_raw.x;
    twist_geo_raw.y = twist_plan_raw.y;
    twist_geo_raw.theta = twist_plan_raw.theta;
    commsgs::geometry_msgs::Twist2D twist_geo = GetThresholdedTwist(twist_geo_raw);

    // Convert geometry_msgs::Twist2D to planning_msgs::Twist2D for twist2Dto3D
    commsgs::planning_msgs::Twist2D twist_plan;
    twist_plan.x = twist_geo.x;
    twist_plan.y = twist_geo.y;
    twist_plan.theta = twist_geo.theta;

    commsgs::geometry_msgs::Twist velocity = twist2Dto3D(twist_plan);

    commsgs::geometry_msgs::PoseStamped transformed_end_pose;
    commsgs::builtin_interfaces::Duration tolerance(
        commsgs::builtin_interfaces::Duration::FromSeconds(costmap_wrapper_->getTransformTolerance()));
    // Use costmap wrapper's transform method instead
    if (!costmap_wrapper_->transformPoseToGlobalFrame(end_pose_, transformed_end_pose)) {
        return false;
    }

    return goal_checkers_[current_goal_checker_]->IsGoalReached(pose.pose, transformed_end_pose.pose, velocity);
}

bool ControllerServer::GetRobotPose(commsgs::geometry_msgs::PoseStamped& pose) {
    commsgs::geometry_msgs::PoseStamped current_pose;
    if (!costmap_wrapper_->getRobotPose(current_pose)) {
        return false;
    }
    pose = current_pose;
    return true;
}

void ControllerServer::SpeedLimitCallback(const commsgs::planning_msgs::SpeedLimit::SharedPtr msg) {
    for (auto it = controllers_.begin(); it != controllers_.end(); ++it) {
        it->second->SetSpeedLimit(msg->speed_limit, msg->percentage);
    }
}

}  // namespace control
}  // namespace autonomy
