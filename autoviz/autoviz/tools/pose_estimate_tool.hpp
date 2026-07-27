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
    return {{"topic", "Topic", "/initialpose",
             {}, common::DisplayPropertyKind::kChannel}};
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
