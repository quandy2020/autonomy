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
#include "autonomy/control/common/controller_exceptions.hpp"
#include "autonomy/common/logging.hpp"

namespace autonomy {
namespace control {

ControllerServer::ControllerServer(const proto::ControllerOptions& options)
    : options_{options}
{
    LOG(INFO) << "Control server init successfully.";
}

ControllerServer::~ControllerServer()
{

}

void ControllerServer::Start()
{
    LOG(INFO) << "start";
}

void ControllerServer::WaitForShutdown()
{

}

bool ControllerServer::FindControllerId(const std::string& c_name, std::string& current_controller)
{
    if (controllers_.find(c_name) == controllers_.end()) {
        if (controllers_.size() == 1 && c_name.empty()) {
            LOG_FIRST_N(WARNING, 1) << "No controller was specified in action call."
                        << " Server will use only plugin loaded " << controller_ids_concat_
                        << ". This warning will appear once.";
        } else {
            LOG(ERROR) << "FollowPath called with controller name " << c_name
                       << ", which does not exist. Available controllers are: "
                       << controller_ids_concat_;
            return false;
        }
    } else {
        DLOG(INFO) << " Selected controller: " << c_name;
        current_controller = c_name;
    }

    return true;
}

bool ControllerServer::FindGoalCheckerId(const std::string& c_name, std::string& current_goal_checker)
{
    if (goal_checkers_.find(c_name) == goal_checkers_.end()) {
        if (goal_checkers_.size() == 1 && c_name.empty()) {
            LOG(WARNING) << "No goal checker was specified in parameter 'current_goal_checker'."
                 << " Server will use only plugin loaded " << goal_checker_ids_concat_
                 << ". This warning will appear once.";
            current_goal_checker = goal_checkers_.begin()->first;
        } else {
            LOG(ERROR) << "FollowPath called with goal_checker name " << c_name
              << " in parameter 'current_goal_checker', which does not exist. Available goal checkers are: "
              << goal_checker_ids_concat_;
            return false;
        }
    } else {
        DLOG(INFO) << "Selected goal checker: " << c_name;
        current_goal_checker = c_name;
    }

    return true;
}

bool ControllerServer::FindProgressCheckerId(const std::string& c_name, std::string& current_progress_checker)
{
    if (progress_checkers_.find(c_name) == progress_checkers_.end()) {
        if (progress_checkers_.size() == 1 && c_name.empty()) {
            LOG(WARNING) << "No progress checker was specified in parameter 'current_progress_checker'."
                    << " Server will use only plugin loaded " << progress_checker_ids_concat_
                    << ". This warning will appear once.";
            current_progress_checker = progress_checkers_.begin()->first;
        } else {
            LOG(ERROR) << "FollowPath called with progress_checker name " << c_name
                << " in parameter 'current_progress_checker', which does not exist. Available progress checkers are: "
                << progress_checker_ids_concat_;
            return false;
        }
    } else {
        DLOG(INFO) << "Selected progress checker: " << c_name;
        current_progress_checker = c_name;
    }

    return true;
}

void ControllerServer::ComputeControl()
{
    LOG(INFO) << "Received a goal, begin computing control effort.";

    // try {
    //     auto goal = action_server_->get_current_goal();
    //     if (!goal) {
    //         return;  //  goal would be nullptr if action_server_ is inactivate.
    //     }

    //     std::string c_name = goal->controller_id;
    //     std::string current_controller;
    //     if (FindControllerId(c_name, current_controller)) {
    //         current_controller_ = current_controller;
    //     } else {
    //         throw common::InvalidController("Failed to find controller name: " + c_name);
    //     }

    //     std::string gc_name = goal->goal_checker_id;
    //     std::string current_goal_checker;
    //     if (FindGoalCheckerId(gc_name, current_goal_checker)) {
    //         current_goal_checker_ = current_goal_checker;
    //     } else {
    //         throw common::ControllerException("Failed to find goal checker name: " + gc_name);
    //     }

    //     std::string pc_name = goal->progress_checker_id;
    //     std::string current_progress_checker;
    //     if (FindProgressCheckerId(pc_name, current_progress_checker)) {
    //         current_progress_checker_ = current_progress_checker;
    //     } else {
    //         throw common::ControllerException("Failed to find progress checker name: " + pc_name);
    //     }

    //     SetPlannerPath(goal->path);
    //     progress_checkers_[current_progress_checker_]->Reset();

    //     last_valid_cmd_time_ = now();
    //     rclcpp::WallRate loop_rate(controller_frequency_);
    //     while (rclcpp::ok()) {
    //     auto start_time = this->now();

    //     if (action_server_ == nullptr || !action_server_->is_server_active()) {
    //         DLOG(INFO) << "Action server unavailable or inactive. Stopping."; 
    //         return;
    //     }

    //     if (action_server_->is_cancel_requested()) {
    //         if (controllers_[current_controller_]->cancel()) {
    //             LOG(INFO) << "Cancellation was successful. Stopping the robot.";
    //             action_server_->terminate_all();
    //             OnGoalExit();
    //             return;
    //         } else {
    //             RCLCPP_INFO_THROTTLE(
    //                 get_logger(), *get_clock(), 1000, "Waiting for the controller to finish cancellation");
    //         }
    //     }
    //         // Don't compute a trajectory until costmap is valid (after clear costmap)
    //         rclcpp::Rate r(100);
    //         auto waiting_start = now();
    //         while (!costmap_wrapper_->isCurrent()) {
    //             if (now() - waiting_start > costmap_update_timeout_) {
    //                 throw common::ControllerTimedOut("Costmap timed out waiting for update");
    //             }
    //             r.sleep();
    //         }

    //         updateGlobalPath();

    //         computeAndPublishVelocity();

    //         if (isGoalReached()) {
    //             LOG(INFO) << "Reached the goal!";
    //             break;
    //         }

    //         auto cycle_duration = this->now() - start_time;
    //         if (!loop_rate.sleep()) {
    //             RCLCPP_WARN(
    //             get_logger(),
    //             "Control loop missed its desired rate of %.4f Hz. Current loop rate is %.4f Hz.",
    //             controller_frequency_, 1 / cycle_duration.seconds());
    //         }
    // }
    // } catch (common::InvalidController& e) {
    //     LOG(ERROR) << e.what();
    //     OnGoalExit();
    //     // std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    //     // result->error_code = Action::Result::INVALID_CONTROLLER;
    //     // action_server_->terminate_current(result);
    //     return;
    // } catch (common::ControllerTFError& e) {
    //     LOG(ERROR) << e.what();
    //     OnGoalExit();
    //     std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    //     result->error_code = Action::Result::TF_ERROR;
    //     action_server_->terminate_current(result);
    //     return;
    // } catch (common::NoValidControl& e) {
    //     LOG(ERROR) << e.what();
    //     OnGoalExit();
    //     // std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    //     // result->error_code = Action::Result::NO_VALID_CONTROL;
    //     // action_server_->terminate_current(result);
    //     return;
    // } catch (common::FailedToMakeProgress& e) {
    //     LOG(ERROR) << e.what();
    //     OnGoalExit();
    //     // std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    //     // result->error_code = Action::Result::FAILED_TO_MAKE_PROGRESS;
    //     // action_server_->terminate_current(result);
    //     return;
    // } catch (common::PatienceExceeded & e) {
    //     LOG(ERROR) << e.what();
    //     OnGoalExit();
    //     // std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    //     // result->error_code = Action::Result::PATIENCE_EXCEEDED;
    //     // action_server_->terminate_current(result);
    //     return;
    // } catch (common::InvalidPath & e) {
    //     LOG(ERROR) << e.what();
    //     OnGoalExit();
    //     // std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    //     // result->error_code = Action::Result::INVALID_PATH;
    //     // action_server_->terminate_current(result);
    //     return;
    // } catch (common::ControllerTimedOut & e) {
    //     LOG(ERROR) << e.what();
    //     OnGoalExit();
    //     // std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    //     // result->error_code = Action::Result::CONTROLLER_TIMED_OUT;
    //     // action_server_->terminate_current(result);
    //     return;
    // } catch (common::ControllerException & e) {
    //     LOG(ERROR) << e.what();
    //     OnGoalExit();
    //     // std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    //     // result->error_code = Action::Result::UNKNOWN;
    //     // action_server_->terminate_current(result);
    //     return;
    // } catch (std::exception & e) {
    //     LOG(ERROR) << e.what();
    //     OnGoalExit();
    //     // std::shared_ptr<Action::Result> result = std::make_shared<Action::Result>();
    //     // result->error_code = Action::Result::UNKNOWN;
    //     // action_server_->terminate_current(result);
    //     return;
    // }

    DLOG(INFO) << "Controller succeeded, setting result";
    OnGoalExit();
}

void ControllerServer::SetPlannerPath(const commsgs::planning_msgs::Path& path)
{
    DLOG(INFO) << "Providing path to the controller " << current_controller_;
    if (path.poses.empty()) {
        throw common::InvalidPath("Path is empty.");
    }
    controllers_[current_controller_]->SetPlan(path);

    end_pose_ = path.poses.back();
    end_pose_.header.frame_id = path.header.frame_id;
    goal_checkers_[current_goal_checker_]->Reset();

    DLOG(INFO) << "Path end point is (" 
          << end_pose_.pose.position.x << ", " 
          << end_pose_.pose.position.y << ")";

    current_path_ = path;
}

void ControllerServer::ComputeAndPublishVelocity()
{
    commsgs::geometry_msgs::PoseStamped pose;

    if (!GetRobotPose(pose)) {
        throw common::ControllerTFError("Failed to obtain robot pose");
    }

    if (!progress_checkers_[current_progress_checker_]->Check(pose)) {
        throw common::FailedToMakeProgress("Failed to make progress");
    }

    // common::geometry_msgs::Twist2D twist = GetThresholdedTwist(odom_sub_->getTwist());

    // commsgs::geometry_msgs::TwistStamped cmd_vel_2d;

    // try {
    //     cmd_vel_2d = controllers_[current_controller_]->ComputeVelocityCommands(
    //         pose,
    //         nav_2d_utils::twist2Dto3D(twist),
    //         goal_checkers_[current_goal_checker_].get());
    //     last_valid_cmd_time_ = now();
    //     cmd_vel_2d.header.frame_id = costmap_wrapper_->getBaseFrameID();
    //     cmd_vel_2d.header.stamp = last_valid_cmd_time_;
    //     // Only no valid control exception types are valid to attempt to have control patience, as
    //     // other types will not be resolved with more attempts
    // } catch (common::NoValidControl & e) {
    //     if (failure_tolerance_ > 0 || failure_tolerance_ == -1.0) {
    //         LOG(WARNING) << e.what();
    //         cmd_vel_2d.twist.angular.x = 0;
    //         cmd_vel_2d.twist.angular.y = 0;
    //         cmd_vel_2d.twist.angular.z = 0;
    //         cmd_vel_2d.twist.linear.x = 0;
    //         cmd_vel_2d.twist.linear.y = 0;
    //         cmd_vel_2d.twist.linear.z = 0;
    //         cmd_vel_2d.header.frame_id = costmap_wrapper_->getBaseFrameID();
    //         cmd_vel_2d.header.stamp = now();
    //         if ((now() - last_valid_cmd_time_).seconds() > failure_tolerance_ &&
    //             failure_tolerance_ != -1.0)
    //         {
    //             throw common::PatienceExceeded("Controller patience exceeded");
    //         }
    //     } else {
    //         throw common::NoValidControl(e.what());
    //     }
    // }

    // std::shared_ptr<Action::Feedback> feedback = std::make_shared<Action::Feedback>();
    // feedback->speed = std::hypot(cmd_vel_2d.twist.linear.x, cmd_vel_2d.twist.linear.y);

    // // Find the closest pose to current pose on global path
    // commsgs::planning_msgs::Path& current_path = current_path_;
    // auto find_closest_pose_idx = [&pose, &current_path]() {
    //     size_t closest_pose_idx = 0;
    //     double curr_min_dist = std::numeric_limits<double>::max();
    //     for (size_t curr_idx = 0; curr_idx < current_path.poses.size(); ++curr_idx) {
    //         double curr_dist = map::costmap_2d::utils::euclidean_distance(pose, current_path.poses[curr_idx]);
    //         if (curr_dist < curr_min_dist) {
    //             curr_min_dist = curr_dist;
    //             closest_pose_idx = curr_idx;
    //         }
    //     }
    //     return closest_pose_idx;
    // };

    // feedback->distance_to_goal = map::costmap_2d::utils::calculate_path_length(current_path_, find_closest_pose_idx());
    // action_server_->publish_feedback(feedback);

    // RCLCPP_DEBUG(get_logger(), "Publishing velocity at time %.2f", now().seconds());
    // PublishVelocity(cmd_vel_2d);
}

void ControllerServer::UpdateGlobalPath()
{
    // if (action_server_->is_preempt_requested()) {
    //     LOG(INFO) << "Passing new path to controller.";
    //     auto goal = action_server_->accept_pending_goal();
    //     std::string current_controller;
    //     if (FindControllerId(goal->controller_id, current_controller)) {
    //         current_controller_ = current_controller;
    //     } else {
    //         RCLCPP_INFO(
    //             get_logger(), "Terminating action, invalid controller %s requested.",
    //             goal->controller_id.c_str());
    //         action_server_->terminate_current();
    //         return;
    //     }
    //     std::string current_goal_checker;
    //     if (FindGoalCheckerId(goal->goal_checker_id, current_goal_checker)) {
    //         current_goal_checker_ = current_goal_checker;
    //     } else {
    //         RCLCPP_INFO(
    //             get_logger(), "Terminating action, invalid goal checker %s requested.",
    //             goal->goal_checker_id.c_str());
    //         action_server_->terminate_current();
    //         return;
    //     }
    //     std::string current_progress_checker;
    //     if (FindProgressCheckerId(goal->progress_checker_id, current_progress_checker)) {
    //     if (current_progress_checker_ != current_progress_checker) {
    //         LOG(INGO) << "Change of progress checker " << progress_checker_id << " requested, resetting it";
    //         current_progress_checker_ = current_progress_checker;
    //         progress_checkers_[current_progress_checker_]->reset();
    //     }
    //     } else {
    //         LOG(INFO) << "Terminating action, invalid progress checker " << goal->progress_checker_id << "requested.";
    //         action_server_->terminate_current();
    //         return;
    //     }
    //     SetPlannerPath(goal->path);
    // }
}

void ControllerServer::PublishVelocity(const commsgs::geometry_msgs::TwistStamped& velocity)
{
    auto cmd_vel = std::make_unique<commsgs::geometry_msgs::TwistStamped>(velocity);
    // if (vel_publisher_->is_activated() && vel_publisher_->get_subscription_count() > 0) {
    //     vel_publisher_->publish(std::move(cmd_vel));
    // }
}

void ControllerServer::PublishZeroVelocity()
{
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

void ControllerServer::OnGoalExit()
{
    if (publish_zero_velocity_) {
        PublishZeroVelocity();
    }

    // Reset the state of the controllers after the task has ended
    for (auto it = controllers_.begin(); it != controllers_.end(); ++it) {
        it->second->Reset();
    }
}

bool ControllerServer::IsGoalReached()
{
    commsgs::geometry_msgs::PoseStamped pose;

    if (!GetRobotPose(pose)) {
        return false;
    }

    return true;

    // commsgs::geometry_msgs::Twist2D twist = GetThresholdedTwist(odom_sub_->getTwist());
    // commsgs::geometry_msgs::Twist velocity = nav_2d_utils::twist2Dto3D(twist);

    commsgs::geometry_msgs::PoseStamped transformed_end_pose;
    // commsgs::builtin_interfaces::Duration tolerance(commsgs::builtin_interfaces::Duration::from_seconds(costmap_wrapper_->getTransformTolerance()));
    // nav_2d_utils::transformPose(
    //     costmap_wrapper_->getTfBuffer(), costmap_wrapper_->getGlobalFrameID(),
    //     end_pose_, transformed_end_pose, tolerance);

    // return goal_checkers_[current_goal_checker_]->IsGoalReached(
    //     pose.pose, transformed_end_pose.pose, velocity);
}

bool ControllerServer::GetRobotPose(commsgs::geometry_msgs::PoseStamped& pose)
{
    commsgs::geometry_msgs::PoseStamped current_pose;
    if (!costmap_wrapper_->getRobotPose(current_pose)) {
        return false;
    }
    pose = current_pose;
    return true;
}

void ControllerServer::SpeedLimitCallback(const commsgs::planning_msgs::SpeedLimit::SharedPtr msg)
{
    for (auto it = controllers_.begin(); it != controllers_.end(); ++it) {
        it->second->SetSpeedLimit(msg->speed_limit, msg->percentage);
    }
}

}  // namespace control
}  // namespace autonomy