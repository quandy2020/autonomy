/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file pose_estimate_tool.hpp
 * @brief 2D Pose Estimate tool — publishes PoseWithCovarianceStamped.
 *
 * RViz @c SetInitialPose / AMCL @c /initialpose equivalent. Extends
 * @ref GoalPoseTool with covariance properties and a green arrow visual.
 * Shortcut key: @c p.
 *
 * @see GoalPoseTool
 * @see NavGoalTool
 * @see PublishMessage
 */

#pragma once

#include "autoviz/tools/goal_pose_tool.hpp"

namespace autoviz {
namespace tools {

/**
 * @class PoseEstimateTool
 * @brief Click-drag pose tool that publishes an initial pose estimate with
 *        configurable XY / yaw covariance.
 *
 * ## Properties
 *
 * | Key            | Label          | Default     |
 * |----------------|----------------|-------------|
 * | topic          | Topic          | /initialpose|
 * | covariance_x   | Covariance x   | 0.25        |
 * | covariance_y   | Covariance y   | 0.25        |
 * | covariance_yaw | Covariance yaw | 0.0685385   |
 *
 * @note Covariance defaults match common RViz / AMCL conventions
 *       (@c yaw ≈ (π/12)²).
 */
class PoseEstimateTool : public GoalPoseTool {
 public:
  /**
   * @brief Stable tool id (delegates to @ref toolId()).
   * @return @c "PoseEstimate".
   */
  std::string id() const override { return toolId(); }

  /**
   * @brief Human-readable toolbar label (delegates to @ref toolLabel()).
   * @return Localized @c "2D Pose Estimate".
   */
  QString label() const override { return toolLabel(); }

  /**
   * @brief Declares Topic and covariance properties.
   * @return Spec list for the tool property editor.
   */
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

  /**
   * @brief Keyboard shortcut for activating this tool.
   * @return @c 'p'.
   */
  char shortcutKey() const override { return 'p'; }

 protected:
  /**
   * @brief Registry id string.
   * @return @c "PoseEstimate".
   */
  std::string toolId() const override { return "PoseEstimate"; }

  /**
   * @brief Toolbar display name.
   * @return Localized @c "2D Pose Estimate".
   */
  QString toolLabel() const override {
    return QStringLiteral("2D Pose Estimate");
  }

  /**
   * @brief Autolink channel from the @c topic property.
   * @return Channel name, default @c /initialpose.
   */
  std::string publishChannel() const override {
    return propertyValue("topic", "/initialpose");
  }

  /**
   * @brief Arrow color (green, RViz PoseTool family).
   * @return @c QColor(0, 178, 0).
   */
  QColor arrowColor() const override { return QColor(0, 178, 0); }

  /**
   * @brief Builds and publishes PoseWithCovarianceStamped on pose commit.
   *
   * @param position Ground-plane world position.
   * @param yaw Yaw about +Z in radians.
   */
  void onPoseSet(const QVector3D& position, float yaw) override;
};

}  // namespace tools
}  // namespace autoviz
