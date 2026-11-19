// Copyright (c) 2023 Open Navigation LLC; Autonomy adaptation 2026.
// SPDX-License-Identifier: Apache-2.0
/** @file path_segments.cpp
 * @brief Native complete-coverage path segments implementation. */
#include "autonomy/task/navigation/coverage/path_segments.hpp"

#include <cmath>
namespace autonomy::task::coverage {
namespace {
void AppendPose(NavPath& path, double x, double y, double z, double yaw) {
  auto* p = path.add_poses();
  p->mutable_header()->CopyFrom(path.header());
  p->mutable_pose()->mutable_position()->set_x(x);
  p->mutable_pose()->mutable_position()->set_y(y);
  p->mutable_pose()->mutable_position()->set_z(z);
  p->mutable_pose()->mutable_orientation()->set_z(std::sin(yaw / 2));
  p->mutable_pose()->mutable_orientation()->set_w(std::cos(yaw / 2));
}
}  // namespace
std::vector<Segment> BuildSegments(const ComputeAction::Result& plan,
                                   size_t max_poses, bool reverse_supported) {
  if (max_poses < 2 || max_poses > 500)
    throw RequestException("Transport segment pose budget must be 2..500");
  if (plan.error_code() != proto::NONE || plan.coverage_path().states().empty())
    throw RequestException("Missing successful coverage path states");
  std::vector<Segment> segments;
  Segment current;
  current.path.mutable_header()->set_frame_id(
      plan.nav_path().header().frame_id());
  auto flush = [&] {
    if (current.path.poses_size() >= 2 && current.length > 1e-8)
      segments.push_back(std::move(current));
    current = Segment();
    current.path.mutable_header()->set_frame_id(
        plan.nav_path().header().frame_id());
  };
  for (const auto& state : plan.coverage_path().states()) {
    if (state.len() < 1e-8) continue;
    if (!std::isfinite(state.x()) || !std::isfinite(state.y()) ||
        !std::isfinite(state.angle()) || !std::isfinite(state.len()) ||
        state.len() < 0 || (state.direction() != 1 && state.direction() != -1))
      throw RequestException("Invalid state in execution path");
    const bool backwards = state.direction() < 0, work = state.type() == 1;
    if (backwards && !reverse_supported)
      throw CoverageException(
          "Configured controller has no reverse execution contract");
    if (current.path.poses_size() > 0 &&
        (current.work != work || current.backward != backwards ||
         std::fabs(current.velocity - state.velocity()) > 1e-6))
      flush();
    current.work = work;
    current.backward = backwards;
    current.velocity = state.velocity();
    if (current.path.poses_size() == 0)
      AppendPose(current.path, state.x(), state.y(), state.z(), state.angle());
    AppendPose(
        current.path,
        state.x() + std::cos(state.angle()) * state.len() * state.direction(),
        state.y() + std::sin(state.angle()) * state.len() * state.direction(),
        state.z(), state.angle());
    current.length += state.len();
    if (static_cast<size_t>(current.path.poses_size()) >= max_poses) flush();
  }
  flush();
  if (segments.empty())
    throw RequestException("No executable coverage segments");
  return segments;
}

}  // namespace autonomy::task::coverage
