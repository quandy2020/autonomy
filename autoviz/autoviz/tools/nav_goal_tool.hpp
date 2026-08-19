/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include "autoviz/tools/goal_pose_tool.hpp"

namespace autoviz {
namespace tools {

class NavGoalTool : public GoalPoseTool {
 public:
  std::string id() const override { return toolId(); }
  QString label() const override { return toolLabel(); }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override {
    return {{"topic", "Topic", "/move_base_simple/goal",
             {}, common::DisplayPropertyKind::kChannel}};
  }
  char shortcutKey() const override { return 'g'; }

 protected:
  std::string toolId() const override { return "NavGoal"; }
  QString toolLabel() const override { return QStringLiteral("2D Nav Goal"); }
  std::string publishChannel() const override {
    return propertyValue("topic", "/move_base_simple/goal");
  }
  QColor arrowColor() const override { return QColor(0, 178, 0); }
  void onPoseSet(const QVector3D& position, float yaw) override;
};

}  // namespace tools
}  // namespace autoviz
