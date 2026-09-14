/*
 * Copyright 2026 The Openbot Authors
 */

#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "autonomy/orbisview/backend/common/stream_envelope.hpp"
#include "autonomy/orbisview/backend/simulation_world/simulation_world_service.hpp"

namespace autonomy {
namespace orbisview {
namespace adapters {

/** Periodic mock channels for browser visualization without Autolink. */
class MockSource {
 public:
  using EmitFn = std::function<void(core::StreamEnvelope)>;

  void SetEmit(EmitFn emit) { emit_ = std::move(emit); }

  std::vector<core::ChannelInfo> Channels() const;

  void Start();
  void Stop();

  void SetNavGoal(double x, double y, double yaw = 0.0);
  void ClearNavGoal();
  bool HasNavGoal(double* x, double* y, double* yaw = nullptr) const;

  void SetCmdVel(double vx, double wz);

  /** Multi-waypoint route (map frame). */
  struct RoutePoint {
    double x{0};
    double y{0};
    double yaw{0};
  };
  void SetRoute(std::vector<RoutePoint> waypoints);
  void ClearRoute();

 private:
  void Loop();
  void EmitJson(const char* channel, const char* schema, const std::string& frame,
                 uint64_t* seq, const std::string& json);

  EmitFn emit_;
  std::atomic<bool> running_{false};
  std::thread thread_;
  uint64_t seq_pose_{0};
  uint64_t seq_path_{0};
  uint64_t seq_map_{0};
  uint64_t seq_costmap_{0};
  uint64_t seq_footprint_{0};
  uint64_t seq_tf_{0};
  uint64_t seq_laser_{0};
  uint64_t seq_image_{0};
  uint64_t seq_cloud_{0};
  uint64_t seq_depth_{0};
  uint64_t seq_explore_{0};
  uint64_t seq_nav_{0};
  uint64_t seq_map_task_{0};
  uint64_t seq_twist_{0};
  uint64_t seq_chassis_{0};
  uint64_t seq_obstacles_{0};
  uint64_t seq_world_{0};
  uint64_t seq_route_{0};
  uint64_t seq_vmap_{0};
  uint64_t seq_pred_{0};
  uint64_t seq_planning_{0};
  uint64_t seq_hmi_{0};
  uint64_t seq_components_{0};
  int tick_{0};

  mutable std::mutex goal_mutex_;
  bool has_goal_{false};
  double goal_x_{2.0};
  double goal_y_{1.0};
  double goal_yaw_{0.0};

  mutable std::mutex motion_mutex_;
  double pose_x_{1.0};
  double pose_y_{0.0};
  double pose_yaw_{0.0};
  double cmd_vx_{0.0};
  double cmd_wz_{0.0};
  std::chrono::steady_clock::time_point cmd_stamp_{};
  bool teleop_active_{false};

  mutable std::mutex route_mutex_;
  std::vector<RoutePoint> route_;

  core::SimulationWorldService world_;
};

}  // namespace adapters
}  // namespace orbisview
}  // namespace autonomy
