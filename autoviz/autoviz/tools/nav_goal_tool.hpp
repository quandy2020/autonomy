/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Combines RViz GoalTool (PoseStamped publish) with Nav2 GoalTool (GoalUpdater).
 *****************************************************************************/

#pragma once

#include <memory>
#include <string>

#include "autolink/node/writer.hpp"
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include "autoviz/tools/goal_pose_tool.hpp"

namespace autoviz {
namespace tools {

/**
 * 2D Goal Pose — click-drag yaw on the ground plane, then:
 *   1. Publish geometry_msgs/PoseStamped on Topic (default /goal_pose).
 *   2. Emit GoalUpdater so Navigation / task panels can react (Nav2 pattern).
 *
 * Matches autonomy::task::kGoalPose and RViz / Nav2 topic conventions.
 */
class NavGoalTool : public GoalPoseTool {
 public:
  std::string id() const override { return toolId(); }
  QString label() const override { return toolLabel(); }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override {
    // RViz GoalTool topic property; default matches autonomy task + Nav2.
    return {{"topic", "Topic", "/goal_pose", {},
             common::DisplayPropertyKind::kChannel}};
  }
  char shortcutKey() const override { return 'g'; }

 protected:
  std::string toolId() const override { return "NavGoal"; }
  QString toolLabel() const override {
    return QStringLiteral("2D Goal Pose");
  }
  std::string publishChannel() const override {
    return propertyValue("topic", "/goal_pose");
  }
  // RViz PoseTool green, slightly darkened for Autoviz background contrast.
  QColor arrowColor() const override { return QColor(0, 178, 0); }
  void onPoseSet(const QVector3D& position, float yaw) override;

 private:
  bool ensureWriter(const std::string& channel);

  std::string writer_channel_;
  std::shared_ptr<::autolink::Writer<automsgs::msgs::geometry_msgs::PoseStamped>>
      writer_;
};

}  // namespace tools
}  // namespace autoviz
