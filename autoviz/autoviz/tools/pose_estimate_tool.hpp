/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include "autoviz/tools/goal_pose_tool.hpp"

namespace autoviz {
namespace tools {

class PoseEstimateTool : public GoalPoseTool {
 public:
  std::string id() const override { return toolId(); }
  QString label() const override { return toolLabel(); }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override {
    return {{"topic", "Topic", "/initialpose", {},
             common::DisplayPropertyKind::kChannel},
            {"covariance_x", "Covariance x", "0.25",
             {}, common::DisplayPropertyKind::kAuto},
            {"covariance_y", "Covariance y", "0.25",
             {}, common::DisplayPropertyKind::kAuto},
            {"covariance_yaw", "Covariance yaw", "0.0685385",
             {}, common::DisplayPropertyKind::kAuto}};
  }
  char shortcutKey() const override { return 'p'; }

 protected:
  std::string toolId() const override { return "PoseEstimate"; }
  QString toolLabel() const override {
    return QStringLiteral("2D Pose Estimate");
  }
  std::string publishChannel() const override {
    return propertyValue("topic", "/initialpose");
  }
  QColor arrowColor() const override { return QColor(0, 255, 0); }
  void onPoseSet(const QVector3D& position, float yaw) override;
};

}  // namespace tools
}  // namespace autoviz
