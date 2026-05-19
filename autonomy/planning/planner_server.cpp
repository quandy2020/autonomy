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

#include "autonomy/planning/planner_server.hpp"

#include "autonomy/common/str_cat.hpp"
#include <unistd.h>  // for getpid()

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <future>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "autonomy/common/config.hpp"
#include "autonomy/common/log.hpp"
#include "autonomy/common/time.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"
#include "autonomy/commsgs/map_msgs.hpp"
#include "autonomy/commsgs/std_msgs.hpp"
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/map/costmap_2d/costmap_2d.hpp"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/map/costmap_2d/utils/occ_grid_values.hpp"
#include "autonomy/planning/common/planner_exceptions.hpp"
#include "autonomy/planning/constants.hpp"
#include "autonomy/planning/planner/navfn/navfn_planner.hpp"

namespace autonomy {
namespace planning {

using Time = commsgs::builtin_interfaces::Time;

PlannerServer::PlannerServer(const proto::PlannerOptions& options)
    : options_{options} {

    AINFO << "PlannerServer created";
    costmap_wrapper_ = std::make_shared<map::costmap_2d::Costmap2DWrapper>(
        options_.costmap(), kCostmapTopicName);
    if (!costmap_wrapper_) {
        AFATAL << "Failed to configure costmap wrapper. costmap_wrapper is "
                  "nullptr";
        return;
    }

    costmap_ = costmap_wrapper_->getCostmap();

    // 初始化 TfBuffer
    tf_ = TfBuffer::Instance();
    if (!tf_) {
        AFATAL << "Failed to get TfBuffer instance";
        return;
    }

    // 设置默认的 planner IDs 和 types
    default_ids_ = {"navfn_planner"};
    default_types_ = {"autonomy::planning::planner::navfn::NavfnPlanner"};
    planner_ids_ = default_ids_;
    planner_types_ = default_types_;

    for (size_t i = 0; i != planner_ids_.size(); i++) {
        common::GlobalPlanner::SharedPtr planner_instance;
        if (planner_ids_[i] == "navfn_planner") {
            planner_instance =
                std::make_shared<::autonomy::planning::planner::navfn::NavfnPlanner>();
        } else {
            AFATAL << "Unknown planner plugin: " << planner_ids_[i];
            return;
        }

        AINFO << "Created planner plugin: " << planner_ids_[i]
              << " of type: " << planner_types_[i];

        // 配置 planner，同时设置 costmap（如果可用）
        std::shared_ptr<map::costmap_2d::Costmap2DWrapper> planner_costmap =
            nullptr;
        if (costmap_wrapper_) {
            planner_costmap = costmap_wrapper_;
        }

        if (!planner_instance->Configure(options_, planner_ids_[i],
                                         planner_costmap)) {
            AFATAL << "Failed to configure planner plugin: " << planner_ids_[i];
            return;
        }

        planners_.insert({planner_ids_[i], planner_instance});
    }

    // 生成 planner_ids_concat_ 字符串
    planner_ids_concat_.clear();
    for (size_t i = 0; i != planner_ids_.size(); i++) {
        if (i > 0) {
            planner_ids_concat_ += " ";
        }
        planner_ids_concat_ += planner_ids_[i];
    }

    AINFO << "Planner Server has " << planners_.size()
          << " planners available: " << planner_ids_concat_;

    // 处理 expected_planner_frequency
    double expected_planner_frequency = options_.expected_planner_frequency();
    if (expected_planner_frequency > 0) {
        max_planner_duration_ = 1 / expected_planner_frequency;
    } else {
        AWARN
            << "The expected planner frequency parameter is "
            << expected_planner_frequency
            << " Hz. The value should be greater than 0.0 to turn on duration "
               "overrun warning messages";
        max_planner_duration_ = 0.0;
    }
    AINFO << "Planning server init successfully.";
}

PlannerServer::~PlannerServer() {
    /*
     * Backstop ensuring this state is destroyed, even if deactivate/cleanup are
     * never called.
     */

    // 停止计算路径的线程
    if (compute_path_to_pose_thread_ &&
        compute_path_to_pose_thread_->joinable()) {
        compute_path_to_pose_thread_->join();
        compute_path_to_pose_thread_.reset();
    }

    if (compute_path_through_poses_thread_ &&
        compute_path_through_poses_thread_->joinable()) {
        compute_path_through_poses_thread_->join();
        compute_path_through_poses_thread_.reset();
    }

    planners_.clear();
}

void PlannerServer::Start() {
    AINFO << "Starting planner server...";

    if (!costmap_wrapper_) {
        AFATAL << "Costmap wrapper is null";
        return;
    }

    // Start the costmap updates
    costmap_wrapper_->Start();

    // Activate planners
    for (auto it = planners_.begin(); it != planners_.end(); ++it) {
        it->second->Activate();
    }

    AINFO << "Planner server started successfully.";
}

void PlannerServer::Shutdown() {
    AINFO << "Planner server shutdown successfully.";
}

commsgs::planning_msgs::Path PlannerServer::GetPlan(
    const commsgs::geometry_msgs::PoseStamped& start,
    const commsgs::geometry_msgs::PoseStamped& goal,
    const std::string& planner_id, std::function<bool()> cancel_checker) {
    commsgs::planning_msgs::Path path;
    AINFO << "Planning algorithm " << planner_id
          << " is trying to find a path from (" << start.pose.position.x << ", "
          << start.pose.position.y << ")"
          << " to "
          << "(" << goal.pose.position.x << "," << goal.pose.position.y << ")";

    uint32_t return_code = 0;
    if (planners_.find(planner_id) != planners_.end()) {
        return_code = planners_[planner_id]->CreatePlan(start, goal, path,
                                                        cancel_checker);
    } else {
        if (planners_.size() == 1 && planner_id.empty()) {
            AWARN
                << "No planners specified in action call. Server will use only "
                   "plugin "
                << planner_ids_concat_
                << " in server. This warning will appear once.";
            return_code = planners_[planners_.begin()->first]->CreatePlan(
                start, goal, path, cancel_checker);
        } else {
            AERROR << "planner " << planner_id << " is not a valid planner. "
                   << "Planner names are: " << planner_ids_concat_;
            throw common::InvalidPlanner("Planner id " + planner_id +
                                         " is invalid");
        }
    }

    if (return_code != 0) {
        AERROR << "planner " << planner_id << " failed to find a path. "
               << "Return code: " << return_code;
        throw common::InvalidPlanner("Planner id " + planner_id +
                                     " failed to find a path");
    }

    return path;
}

void PlannerServer::WaitForCostmap() {
    // 如果未配置超时时间，则一直等待直到 costmap 当前
    const double timeout_sec = options_.costmap_update_timeout();
    const bool use_timeout = timeout_sec > 0.0;

    AINFO << "Waiting for global map (Costmap2D) to become ready"
          << (use_timeout ? ::autonomy::common::StrCat(" (timeout = ", timeout_sec, " s)")
                          : " (no timeout)");

    const auto start_time = std::chrono::steady_clock::now();

    while (true) {
        // 检查 costmap 是否准备好
        if (costmap_wrapper_ && costmap_wrapper_->isCurrent()) {
            costmap_received_.store(true, std::memory_order_release);
            break;
        }

        if (use_timeout) {
            const auto now = std::chrono::steady_clock::now();
            const auto elapsed_duration =
                std::chrono::duration_cast<std::chrono::duration<double>>(
                    now - start_time);
            const double elapsed = elapsed_duration.count();
            if (elapsed > timeout_sec) {
                AWARN << "WaitForCostmap timeout: global map is still not "
                         "confirmed "
                         "ready after "
                      << elapsed << " seconds.";
                break;
            }
        }

        // 以较小频率轮询，避免忙等
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    AINFO << "WaitForCostmap finished (either timeout or map assumed ready).";
}

bool PlannerServer::GetRobotPose(commsgs::geometry_msgs::PoseStamped& pose) {
    if (!costmap_wrapper_) {
        AERROR << "Costmap wrapper is null, cannot get robot pose";
        return false;
    }

    return costmap_wrapper_->getRobotPose(pose);
}

bool PlannerServer::TransformPosesToGlobalFrame(
    commsgs::geometry_msgs::PoseStamped& curr_start,
    commsgs::geometry_msgs::PoseStamped& curr_goal) {
    if (!costmap_wrapper_) {
        AERROR << "Costmap wrapper is null, cannot transform poses";
        return false;
    }

    commsgs::geometry_msgs::PoseStamped transformed_start;
    commsgs::geometry_msgs::PoseStamped transformed_goal;

    if (!costmap_wrapper_->transformPoseToGlobalFrame(curr_start,
                                                      transformed_start) ||
        !costmap_wrapper_->transformPoseToGlobalFrame(curr_goal,
                                                      transformed_goal)) {
        AERROR << "Failed to transform poses to global frame";
        return false;
    }

    curr_start = transformed_start;
    curr_goal = transformed_goal;
    return true;
}

bool PlannerServer::ValidatePath(
    const commsgs::geometry_msgs::PoseStamped& curr_goal,
    const commsgs::planning_msgs::Path& path, const std::string& planner_id) {
    if (path.poses.empty()) {
        AWARN << "Planning algorithm " << planner_id
              << " failed to generate a valid path to ("
              << curr_goal.pose.position.x << ", " << curr_goal.pose.position.y
              << ")";
        return false;
    }

    AINFO << "Found valid path of size " << path.poses.size() << " to ("
          << curr_goal.pose.position.x << ", " << curr_goal.pose.position.y
          << ")";
    return true;
}

void PlannerServer::ComputePlan() {
    // Autolink action server removed; path planning is invoked via GetPlan().
    AINFO << "ComputePlan: no action server (stub).";
}

void PlannerServer::PublishPlan(const commsgs::planning_msgs::Path& path) {
    if (path.poses.empty()) {
        AWARN << "Cannot publish empty path";
        return;
    }

    AINFO << "Published plan with " << path.poses.size()
          << " poses to topic: " << kPlanTopicName;
}

void PlannerServer::ExceptionWarning(
    const commsgs::geometry_msgs::PoseStamped& start,
    const commsgs::geometry_msgs::PoseStamped& goal,
    const std::string& planner_id, const std::exception& ex) {
    AWARN << ::autonomy::common::StrCat(planner_id, " plugin failed to plan from (",
                          start.pose.position.x, start.pose.position.y,
                          ") to (", goal.pose.position.x, goal.pose.position.y,
                          "): ", ex.what());
}

}  // namespace planning
}  // namespace autonomy