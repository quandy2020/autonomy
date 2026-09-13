/*
 * Copyright 2026 The Openbot Authors
 *
 * Aggregates multi-channel robot state into orbisview.render.WorldState JSON
 * (Dreamview SimulationWorld-inspired, OrbisView-native).
 */

#pragma once

#include <mutex>
#include <sstream>
#include <string>
#include <vector>

namespace autonomy {
namespace orbisview {
namespace core {

struct WorldPose {
  double x{0};
  double y{0};
  double yaw{0};
  bool valid{false};
};

struct WorldChassis {
  double vx{0};
  double wz{0};
  std::string gear{"N"};
  double throttle{0};
  double brake{0};
  double steering{0};
  std::string driving_mode{"MANUAL"};
  bool valid{false};
};

struct WorldObstacle {
  int id{0};
  double x{0};
  double y{0};
  double yaw{0};
  double length{0.6};
  double width{0.4};
  std::string type{"UNKNOWN"};
  double vx{0};
  double vy{0};
};

struct WorldGoal {
  double x{0};
  double y{0};
  bool valid{false};
};

class SimulationWorldService {
 public:
  void SetPose(double x, double y, double yaw);
  void SetChassis(const WorldChassis& c);
  void SetObstacles(std::vector<WorldObstacle> obs);
  void SetPathPoses(std::vector<WorldPose> poses);
  void SetGoal(double x, double y);
  void ClearGoal();

  /** Compact WorldState JSON for StreamEnvelope payload. */
  std::string ToJson() const;

 private:
  mutable std::mutex mutex_;
  WorldPose pose_;
  WorldChassis chassis_;
  std::vector<WorldObstacle> obstacles_;
  std::vector<WorldPose> path_;
  WorldGoal goal_;
};

}  // namespace core
}  // namespace orbisview
}  // namespace autonomy
