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

#include <absl/strings/str_cat.h>
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

#include "autolink/autolink.hpp"
#include "autolink/class_loader/class_loader_manager.hpp"
#include "autolink/common/log.hpp"
#include "autonomy/common/config.hpp"
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

namespace autonomy {
namespace planning {

using Time = commsgs::builtin_interfaces::Time;

PlannerServer::PlannerServer(const proto::PlannerOptions& options) : options_{options} {
  // 创建用于订阅全局代价地图的 autolink 节点（与 MapServer 解耦）
  // 添加进程ID以确保节点名唯一（避免节点名称冲突）
  std::string actual_node_name = std::string(kMapNodeName) + "_" + std::to_string(getpid());
  node_ = ::autolink::CreateNode(actual_node_name);
  AINFO << "PlannerServer created with node name: " << actual_node_name;
  costmap_wrapper_ = std::make_shared<map::costmap_2d::Costmap2DWrapper>(options_.costmap(), kCostmapTopicName);
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
  default_types_ = {"autonomy::planning::plugins::navfn::NavfnPlanner"};
  planner_ids_ = default_ids_;
  planner_types_ = default_types_;

  // Helper function to get library path for a plugin
  auto GetPluginLibraryPath = [](const std::string& plugin_name) -> std::string {
    // Try install directory first
    std::string install_path =
        std::string(::autonomy::common::kLibraryInstallDir) + "/lib/libautonomy_planning_" + plugin_name + ".so";
    if (autolink::common::PathExists(install_path)) {
      return install_path;
    }
    // Try build directory
    std::string build_path =
        std::string(::autonomy::common::kLibraryBuildDir) + "/lib/libautonomy_planning_" + plugin_name + ".so";
    if (autolink::common::PathExists(build_path)) {
      return build_path;
    }
    return "";
  };

  for (size_t i = 0; i != planner_ids_.size(); i++) {
    std::string library_path = GetPluginLibraryPath(planner_ids_[i]);

    // 使用 ClassLoaderManager 加载插件
    common::GlobalPlanner::SharedPtr planner;
    static autolink::class_loader::ClassLoaderManager loader_manager;

    // 如果提供了库路径，先加载库
    if (!library_path.empty()) {
      loader_manager.LoadLibrary(library_path);
    }

    // 使用 ClassLoaderManager 创建插件实例
    planner = loader_manager.CreateClassObj<common::GlobalPlanner>(planner_types_[i]);

    if (!planner) {
      AFATAL << "Failed to create planner plugin: " << planner_ids_[i] << " of type: " << planner_types_[i];
      return;
    }

    AINFO << "Created planner plugin: " << planner_ids_[i] << " of type: " << planner_types_[i];

    // 配置 planner，同时设置 costmap（如果可用）
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> planner_costmap = nullptr;
    if (costmap_wrapper_) {
      planner_costmap = costmap_wrapper_;
    }

    if (!planner->Configure(options_, planner_ids_[i], planner_costmap)) {
      AFATAL << "Failed to configure planner plugin: " << planner_ids_[i];
      return;
    }

    planners_.insert({planner_ids_[i], planner});
  }

  // 生成 planner_ids_concat_ 字符串
  planner_ids_concat_.clear();
  for (size_t i = 0; i != planner_ids_.size(); i++) {
    if (i > 0) {
      planner_ids_concat_ += " ";
    }
    planner_ids_concat_ += planner_ids_[i];
  }

  AINFO << "Planner Server has " << planners_.size() << " planners available: " << planner_ids_concat_;

  // 创建路径发布者
  path_publisher_ = node_->CreateWriter<commsgs::planning_msgs::Path>(kPlanTopicName);
  if (!path_publisher_) {
    AWARN << "Failed to create path publisher for topic: " << kPlanTopicName;
  } else {
    AINFO << "Path publisher created for topic: " << kPlanTopicName;
  }

  // 处理 expected_planner_frequency
  double expected_planner_frequency = options_.expected_planner_frequency();
  if (expected_planner_frequency > 0) {
    max_planner_duration_ = 1 / expected_planner_frequency;
  } else {
    AWARN << "The expected planner frequency parameter is " << expected_planner_frequency
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
  if (compute_path_to_pose_thread_ && compute_path_to_pose_thread_->joinable()) {
    compute_path_to_pose_thread_->join();
    compute_path_to_pose_thread_.reset();
  }

  if (compute_path_through_poses_thread_ && compute_path_through_poses_thread_->joinable()) {
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

  // Create action server (ComputePathToPose)
  try {
    action_server_ =
        std::make_shared<ActionServer>(node_, "compute_path_to_pose", std::bind(&PlannerServer::ComputePlan, this),
                                       /*completion_callback=*/nullptr, std::chrono::milliseconds(500),
                                       /*realtime=*/false);
  } catch (const std::exception& ex) {
    AFATAL << "Failed to create planner action server: " << ex.what();
    return;
  }

  action_server_->Activate();
  AINFO << "Planner server started successfully.";
}

void PlannerServer::Shutdown() {
  AINFO << "Shutting down planner server...";

  if (action_server_) {
    action_server_->Deactivate();
    action_server_.reset();
  }

  for (auto it = planners_.begin(); it != planners_.end(); ++it) {
    it->second->Deactivate();
  }

  if (costmap_wrapper_) {
    costmap_wrapper_->Stop();
  }

  AINFO << "Planner server shutdown successfully.";
}

commsgs::planning_msgs::Path PlannerServer::GetPlan(const commsgs::geometry_msgs::PoseStamped& start,
                                                    const commsgs::geometry_msgs::PoseStamped& goal,
                                                    const std::string& planner_id,
                                                    std::function<bool()> cancel_checker) {
  commsgs::planning_msgs::Path path;
  AINFO << "Planning algorithm " << planner_id << " is trying to find a path from (" << start.pose.position.x << ", "
        << start.pose.position.y << ")"
        << " to "
        << "(" << goal.pose.position.x << "," << goal.pose.position.y << ")";

  uint32_t return_code = 0;
  if (planners_.find(planner_id) != planners_.end()) {
    return_code = planners_[planner_id]->CreatePlan(start, goal, path, cancel_checker);
  } else {
    if (planners_.size() == 1 && planner_id.empty()) {
      AWARN << "No planners specified in action call. Server will use only "
               "plugin "
            << planner_ids_concat_ << " in server. This warning will appear once.";
      return_code = planners_[planners_.begin()->first]->CreatePlan(start, goal, path, cancel_checker);
    } else {
      AERROR << "planner " << planner_id << " is not a valid planner. "
             << "Planner names are: " << planner_ids_concat_;
      throw common::InvalidPlanner("Planner id " + planner_id + " is invalid");
    }
  }

  if (return_code != 0) {
    AERROR << "planner " << planner_id << " failed to find a path. "
           << "Return code: " << return_code;
    throw common::InvalidPlanner("Planner id " + planner_id + " failed to find a path");
  }

  return path;
}

void PlannerServer::WaitForCostmap() {
  // 如果未配置超时时间，则一直等待直到 costmap 当前
  const double timeout_sec = options_.costmap_update_timeout();
  const bool use_timeout = timeout_sec > 0.0;

  AINFO << "Waiting for global map (Costmap2D) to become ready"
        << (use_timeout ? absl::StrCat(" (timeout = ", timeout_sec, " s)") : " (no timeout)");

  const auto start_time = std::chrono::steady_clock::now();

  while (true) {
    // 检查 costmap 是否准备好
    if (costmap_wrapper_ && costmap_wrapper_->isCurrent()) {
      costmap_received_.store(true, std::memory_order_release);
      break;
    }

    if (use_timeout) {
      const auto now = std::chrono::steady_clock::now();
      const auto elapsed_duration = std::chrono::duration_cast<std::chrono::duration<double>>(now - start_time);
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

bool PlannerServer::TransformPosesToGlobalFrame(commsgs::geometry_msgs::PoseStamped& curr_start,
                                                commsgs::geometry_msgs::PoseStamped& curr_goal) {
  if (!costmap_wrapper_) {
    AERROR << "Costmap wrapper is null, cannot transform poses";
    return false;
  }

  commsgs::geometry_msgs::PoseStamped transformed_start;
  commsgs::geometry_msgs::PoseStamped transformed_goal;

  if (!costmap_wrapper_->transformPoseToGlobalFrame(curr_start, transformed_start) ||
      !costmap_wrapper_->transformPoseToGlobalFrame(curr_goal, transformed_goal)) {
    AERROR << "Failed to transform poses to global frame";
    return false;
  }

  curr_start = transformed_start;
  curr_goal = transformed_goal;
  return true;
}

bool PlannerServer::ValidatePath(const commsgs::geometry_msgs::PoseStamped& curr_goal,
                                 const commsgs::planning_msgs::Path& path, const std::string& planner_id) {
  if (path.poses.empty()) {
    AWARN << "Planning algorithm " << planner_id << " failed to generate a valid path to (" << curr_goal.pose.position.x
          << ", " << curr_goal.pose.position.y << ")";
    return false;
  }

  AINFO << "Found valid path of size " << path.poses.size() << " to (" << curr_goal.pose.position.x << ", "
        << curr_goal.pose.position.y << ")";
  return true;
}

void PlannerServer::ComputePlan() {
  AINFO << "Compute plan processing thread started.";

  // One-shot action callback executed in SimpleActionServer worker thread.
  if (!action_server_) {
    AWARN << "Action server is not available";
    return;
  }

  auto goal = action_server_->GetCurrentGoal();
  if (!goal) {
    // Could be inactive or preempted
    return;
  }

  auto start_time = Time::Now();
  auto result = std::make_shared<Action::Result>();

  commsgs::geometry_msgs::PoseStamped start;
  commsgs::geometry_msgs::PoseStamped goal_pose;

  try {
    if (action_server_->IsCancelRequested()) {
      result->set_error_code(
          autonomy::tasks::behavior_tree::proto::ComputePathToPoseErrorCode::COMPUTE_PATH_TO_POSE_ERROR_NONE);
      result->set_error_msg("");
      action_server_->TerminateCurrent(result);
      return;
    }

    // Wait for costmap readiness
    WaitForCostmap();
    if (!costmap_wrapper_ || !costmap_wrapper_->isCurrent()) {
      throw common::PlannerTimedOut("Costmap timed out waiting for update");
    }

    // Use start pose if provided otherwise use current robot pose
    if (goal->use_start()) {
      start = commsgs::geometry_msgs::FromProto(goal->start());
    } else {
      if (!GetRobotPose(start)) {
        throw common::PlannerTFError("Unable to get start pose");
      }
    }

    goal_pose = commsgs::geometry_msgs::FromProto(goal->goal());

    // Transform poses into global frame used by costmap
    if (!TransformPosesToGlobalFrame(start, goal_pose)) {
      throw common::PlannerTFError("Unable to transform poses to global frame");
    }

    // Basic bounds / occupancy validation (mirrors Nav2 behavior at a high-level)
    if (!costmap_) {
      throw common::PlannerException("Costmap is null");
    }
    {
      std::unique_lock<map::costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

      unsigned int mx = 0, my = 0;
      if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx, my)) {
        throw common::StartOutsideMapBounds("Start is outside map bounds");
      }
      unsigned char start_cost = costmap_->getCost(mx, my);
      if (start_cost == map::costmap_2d::LETHAL_OBSTACLE ||
          start_cost == map::costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        throw common::StartOccupied("Start is occupied");
      }

      if (!costmap_->worldToMap(goal_pose.pose.position.x, goal_pose.pose.position.y, mx, my)) {
        throw common::GoalOutsideMapBounds("Goal is outside map bounds");
      }
      unsigned char goal_cost = costmap_->getCost(mx, my);
      if (goal_cost == map::costmap_2d::LETHAL_OBSTACLE || goal_cost == map::costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        throw common::GoalOccupied("Goal is occupied");
      }
    }

    auto cancel_checker = [this]() { return action_server_ && action_server_->IsCancelRequested(); };

    // Compute path using selected planner
    commsgs::planning_msgs::Path path = GetPlan(start, goal_pose, goal->planner_id(), cancel_checker);

    if (!ValidatePath(goal_pose, path, goal->planner_id())) {
      throw common::NoValidPathCouldBeFound(goal->planner_id() + " generated an empty path");
    }

    // Publish for visualization (autolink topic: /plan)
    PublishPlan(path);

    // Fill action result (proto)
    auto* proto_path = result->mutable_path();
    *proto_path->mutable_header() = commsgs::std_msgs::ToProto(path.header);
    for (const auto& pose : path.poses) {
      *proto_path->add_poses() = commsgs::geometry_msgs::ToProto(pose);
    }

    auto cycle_duration = Time::Now() - start_time;
    *result->mutable_planning_time() = commsgs::builtin_interfaces::ToProto(cycle_duration);

    result->set_error_code(
        autonomy::tasks::behavior_tree::proto::ComputePathToPoseErrorCode::COMPUTE_PATH_TO_POSE_ERROR_NONE);
    result->set_error_msg("");

    if (max_planner_duration_ > 0.0 && cycle_duration.Seconds() > max_planner_duration_) {
      AWARN << "Planner loop missed its desired rate of " << (1.0 / max_planner_duration_)
            << " Hz. Current loop rate is " << (1.0 / cycle_duration.Seconds()) << " Hz";
    }

    action_server_->SucceededCurrent(result);
  } catch (common::InvalidPlanner& ex) {
    ExceptionWarning(start, goal_pose, goal->planner_id(), ex);
    result->set_error_code(
        autonomy::tasks::behavior_tree::proto::ComputePathToPoseErrorCode::COMPUTE_PATH_TO_POSE_ERROR_INVALID_PLANNER);
    result->set_error_msg(ex.what());
    action_server_->TerminateCurrent(result);
  } catch (common::StartOccupied& ex) {
    ExceptionWarning(start, goal_pose, goal->planner_id(), ex);
    result->set_error_code(
        autonomy::tasks::behavior_tree::proto::ComputePathToPoseErrorCode::COMPUTE_PATH_TO_POSE_ERROR_START_OCCUPIED);
    result->set_error_msg(ex.what());
    action_server_->TerminateCurrent(result);
  } catch (common::GoalOccupied& ex) {
    ExceptionWarning(start, goal_pose, goal->planner_id(), ex);
    result->set_error_code(
        autonomy::tasks::behavior_tree::proto::ComputePathToPoseErrorCode::COMPUTE_PATH_TO_POSE_ERROR_GOAL_OCCUPIED);
    result->set_error_msg(ex.what());
    action_server_->TerminateCurrent(result);
  } catch (common::NoValidPathCouldBeFound& ex) {
    ExceptionWarning(start, goal_pose, goal->planner_id(), ex);
    result->set_error_code(
        autonomy::tasks::behavior_tree::proto::ComputePathToPoseErrorCode::COMPUTE_PATH_TO_POSE_ERROR_NO_VALID_PATH);
    result->set_error_msg(ex.what());
    action_server_->TerminateCurrent(result);
  } catch (common::PlannerTimedOut& ex) {
    ExceptionWarning(start, goal_pose, goal->planner_id(), ex);
    result->set_error_code(
        autonomy::tasks::behavior_tree::proto::ComputePathToPoseErrorCode::COMPUTE_PATH_TO_POSE_ERROR_TIMEOUT);
    result->set_error_msg(ex.what());
    action_server_->TerminateCurrent(result);
  } catch (common::StartOutsideMapBounds& ex) {
    ExceptionWarning(start, goal_pose, goal->planner_id(), ex);
    result->set_error_code(autonomy::tasks::behavior_tree::proto::ComputePathToPoseErrorCode::
                               COMPUTE_PATH_TO_POSE_ERROR_START_OUTSIDE_MAP);
    result->set_error_msg(ex.what());
    action_server_->TerminateCurrent(result);
  } catch (common::GoalOutsideMapBounds& ex) {
    ExceptionWarning(start, goal_pose, goal->planner_id(), ex);
    result->set_error_code(
        autonomy::tasks::behavior_tree::proto::ComputePathToPoseErrorCode::COMPUTE_PATH_TO_POSE_ERROR_GOAL_OUTSIDE_MAP);
    result->set_error_msg(ex.what());
    action_server_->TerminateCurrent(result);
  } catch (common::PlannerTFError& ex) {
    ExceptionWarning(start, goal_pose, goal->planner_id(), ex);
    result->set_error_code(
        autonomy::tasks::behavior_tree::proto::ComputePathToPoseErrorCode::COMPUTE_PATH_TO_POSE_ERROR_TF_ERROR);
    result->set_error_msg(ex.what());
    action_server_->TerminateCurrent(result);
  } catch (common::PlannerCancelled& ex) {
    result->set_error_code(
        autonomy::tasks::behavior_tree::proto::ComputePathToPoseErrorCode::COMPUTE_PATH_TO_POSE_ERROR_NONE);
    result->set_error_msg(ex.what());
    action_server_->TerminateAll(result);
  } catch (std::exception& ex) {
    ExceptionWarning(start, goal_pose, goal->planner_id(), ex);
    result->set_error_code(
        autonomy::tasks::behavior_tree::proto::ComputePathToPoseErrorCode::COMPUTE_PATH_TO_POSE_ERROR_UNKNOWN);
    result->set_error_msg(ex.what());
    action_server_->TerminateCurrent(result);
  }

  AINFO << "Compute plan processing thread stopped.";
}

void PlannerServer::PublishPlan(const commsgs::planning_msgs::Path& path) {
  if (!path_publisher_) {
    AWARN << "Path publisher is not available, cannot publish plan";
    return;
  }

  if (path.poses.empty()) {
    AWARN << "Cannot publish empty path";
    return;
  }

  // 创建共享指针的消息用于发布
  auto path_msg = std::make_shared<commsgs::planning_msgs::Path>(path);
  if (path_publisher_->Write(path_msg)) {
    AINFO << "Published plan with " << path.poses.size() << " poses to topic: " << kPlanTopicName;
  } else {
    AWARN << "Failed to publish plan to topic: " << kPlanTopicName;
  }
}

void PlannerServer::ExceptionWarning(const commsgs::geometry_msgs::PoseStamped& start,
                                     const commsgs::geometry_msgs::PoseStamped& goal, const std::string& planner_id,
                                     const std::exception& ex) {
  AWARN << absl::StrCat(planner_id, " plugin failed to plan from (", start.pose.position.x, start.pose.position.y,
                        ") to (", goal.pose.position.x, goal.pose.position.y, "): ", ex.what());
}

}  // namespace planning
}  // namespace autonomy