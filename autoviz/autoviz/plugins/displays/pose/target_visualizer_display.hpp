/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <chrono>
#include <memory>
#include <string>

#include <QColor>
#include <QObject>
#include <QQuaternion>
#include <QVector3D>

#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/plugins/displays/channel_display.hpp"

namespace autoviz {
namespace display {

class TargetVisualizerDisplay
    : public ChannelDisplay<automsgs::msgs::geometry_msgs::PoseStamped> {
 public:
  explicit TargetVisualizerDisplay(std::string channel);
  ~TargetVisualizerDisplay() override;

  std::string typeId() const override { return "TargetVisualizer"; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  void onEnable() override;
  void onDisable() override;
  void processMessage(
      const automsgs::msgs::geometry_msgs::PoseStamped& message) override;
  void clearReceivedData() override;
  void onUpdate() override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  void setGoalPose(double x, double y, double yaw, const std::string& frame);
  void resetArrivalState();
  bool robotNearGoal(float distance_m) const;
  void clearDrawnGeometry(rendering::SceneOverlay& scene) const;

  bool have_pose_ = false;
  QVector3D position_;
  QQuaternion orientation_;
  /** Receives NavGoalTool GoalUpdater broadcasts (channel backup). */
  std::unique_ptr<QObject> goal_bridge_;

  /** Arrival success morph: double-ring → single expanded green ring. */
  bool arrived_ = false;
  std::chrono::steady_clock::time_point arrival_t0_{};
};

}  // namespace display
}  // namespace autoviz
