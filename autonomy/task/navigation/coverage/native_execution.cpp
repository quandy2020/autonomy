// Copyright (c) 2023 Open Navigation LLC; Autonomy adaptation 2026.
// SPDX-License-Identifier: Apache-2.0
/** @file native_execution.cpp
 * @brief Native complete-coverage native execution implementation. */
#include "autonomy/task/navigation/coverage/native_execution.hpp"

#include <cmath>

#include "autonomy/task/navigation/coverage/path_segments.hpp"
namespace autonomy::task::coverage {
NativeExecution::NativeExecution(
    Configuration c, std::shared_ptr<CoverageServer> planner,
    navigation::NavigationClient::Ptr nav, NavigateAction::Goal goal,
    std::shared_ptr<common::MotionDispatchGate> gate, Tool tool,
    std::function<void(const ComputeAction::Result&)> plan_ready)
    : config_(std::move(c)),
      planner_(std::move(planner)),
      navigation_(std::move(nav)),
      goal_(std::move(goal)),
      gate_(std::move(gate)),
      tool_(std::move(tool)),
      plan_ready_(std::move(plan_ready)) {}
BT::NodeStatus NativeExecution::Fail(proto::ErrorCode error,
                                     const std::string& message) {
  error_ = error;
  message_ = message;
  Cancel();
  return BT::NodeStatus::FAILURE;
}
BT::NodeStatus NativeExecution::Compute() {
  if (computed_)
    return error_ == proto::NONE ? BT::NodeStatus::SUCCESS
                                 : BT::NodeStatus::FAILURE;
  computed_ = true;
  if (!gate_ || !gate_->PermitValid())
    return Fail(proto::CANCELED, "Coverage movement permit revoked");
  if (goal_.request().has_coordinates_in_cartesian_frame() &&
      !goal_.request().coordinates_in_cartesian_frame())
    return Fail(proto::INVALID_REQUEST,
                "Navigation requires Cartesian coordinates");
  plan_ = planner_->Compute(goal_.request(),
                            [this] { return !gate_->PermitValid(); });
  if (plan_.error_code() != proto::NONE)
    return Fail(plan_.error_code(), plan_.error_msg());
  if (plan_ready_) plan_ready_(plan_);
  try {
    segments_ = BuildSegments(plan_, config_.max_transport_poses,
                              config_.controller_supports_reverse);
  } catch (const CoverageException& e) {
    return Fail(proto::CONTROLLER_UNSUPPORTED, e.what());
  } catch (const std::exception& e) {
    return Fail(proto::INVALID_REQUEST, e.what());
  }
  for (const auto& segment : segments_) total_length_ += segment.length;
  return BT::NodeStatus::SUCCESS;
}
BT::NodeStatus NativeExecution::Follow(const Segment& segment) {
  if (!gate_->PermitValid())
    return Fail(proto::CANCELED, "Coverage motion permit revoked");
  auto& client = navigation_->follow_path_client();
  if (!sent_) {
    if (!client.ActionServerIsReady()) return BT::NodeStatus::RUNNING;
    if (tool_ && !tool_(segment.work, segment.velocity))
      return Fail(proto::TOOL_FAILURE, "Tool command was not acknowledged");
    automsgs::actions::FollowPathAction::Goal request;
    *request.mutable_path() = segment.path;
    request.set_controller_id(goal_.controller_id().empty()
                                  ? config_.controller_id
                                  : goal_.controller_id());
    if (!motion_.Begin(client, request, gate_))
      return Fail(proto::CANCELED, "Coverage send admission revoked");
    sent_ = true;
  }
  auto status =
      motion_.Tick([this] { return !gate_->PermitValid(); }, [](const auto&) {},
                   [this](int, const auto& msg) {
                     error_ = proto::INTERNAL_F2C_ERROR;
                     message_ = msg;
                   });
  if (status != BT::NodeStatus::RUNNING) {
    sent_ = false;
    if (tool_) tool_(false, 0);
  }
  return status;
}
BT::NodeStatus NativeExecution::NavigateToStart() {
  if (approached_) return BT::NodeStatus::SUCCESS;
  if (!goal_.navigate_to_start()) {
    approached_ = true;
    return BT::NodeStatus::SUCCESS;
  }
  if (!approach_planned_) {
    if (!navigation_->IsPlanningReady()) return BT::NodeStatus::RUNNING;
    NavPath path;
    int error = 0;
    std::string message;
    if (!navigation_->ComputePathToPose(plan_.nav_path().poses(0), config_.planner_id, path,
                                        &error, &message))
      return Fail(proto::INTERNAL_F2C_ERROR,
                  "Approach planning failed: " + message);
    if (path.poses_size() < 2) {
      approached_ = true;
      return BT::NodeStatus::SUCCESS;
    }
    Segment segment;
    segment.path.mutable_header()->CopyFrom(path.header());
    for (int i = 0; i < path.poses_size(); ++i) {
      *segment.path.add_poses() = path.poses(i);
      if (i > 0) {
        const auto& a = path.poses(i - 1).pose().position();
        const auto& b = path.poses(i).pose().position();
        segment.length += std::hypot(b.x() - a.x(), b.y() - a.y());
      }
      if (segment.path.poses_size() ==
              static_cast<int>(config_.max_transport_poses) ||
          i + 1 == path.poses_size()) {
        if (segment.path.poses_size() > 1) approach_.push_back(segment);
        segment = Segment();
        segment.path.mutable_header()->CopyFrom(path.header());
        *segment.path.add_poses() = path.poses(i);
      }
    }
    approach_planned_ = true;
  }
  if (approach_index_ >= approach_.size()) {
    approached_ = true;
    return BT::NodeStatus::SUCCESS;
  }
  const auto status = Follow(approach_[approach_index_]);
  if (status == BT::NodeStatus::SUCCESS) {
    ++approach_index_;
    return BT::NodeStatus::RUNNING;
  }
  return status;
}
BT::NodeStatus NativeExecution::FollowCoverage() {
  if (index_ >= segments_.size()) return BT::NodeStatus::SUCCESS;
  const auto status = Follow(segments_[index_]);
  if (status == BT::NodeStatus::SUCCESS) {
    ++index_;
    return index_ >= segments_.size() ? BT::NodeStatus::SUCCESS
                                      : BT::NodeStatus::RUNNING;
  }
  return status;
}
void NativeExecution::Cancel() {
  if (gate_) gate_->Revoke();
  if (navigation_) motion_.Cancel(navigation_->follow_path_client());
  if (tool_) tool_(false, 0);
}
double NativeExecution::DistanceRemaining() const {
  double result = 0;
  for (size_t i = index_; i < segments_.size(); ++i)
    result += segments_[i].length;
  return result;
}
double NativeExecution::ProgressValue() const {
  return total_length_ > 0 ? 1 - DistanceRemaining() / total_length_ : 0;
}

}  // namespace autonomy::task::coverage
