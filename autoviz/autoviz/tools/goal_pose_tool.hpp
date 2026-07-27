/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <optional>

#include <QColor>
#include <QMouseEvent>
#include <QVector3D>

#include "autoviz/common/tool.hpp"

namespace autoviz {
namespace rendering {
class SceneOverlay;
}

namespace tools {

/** RViz PoseTool clone — click-drag-release pose on ground plane (Nav Goal / Pose Estimate). */
class GoalPoseTool : public common::Tool {
 public:
  void activate(common::ToolContext* context) override;
  void deactivate() override;
  bool mousePressEvent(QMouseEvent* event) override;
  bool mouseMoveEvent(QMouseEvent* event) override;
  bool mouseReleaseEvent(QMouseEvent* event) override;
  void onDraw(rendering::SceneOverlay& scene) override;
  QString statusText() const override;

 protected:
  virtual std::string toolId() const = 0;
  virtual QString toolLabel() const = 0;
  virtual std::string publishChannel() const = 0;
  virtual QColor arrowColor() const = 0;
  virtual void onPoseSet(const QVector3D& position, float yaw) = 0;

 private:
  enum class State { kPosition, kOrientation };

  bool pickGround(int x, int y, QVector3D* hit) const;
  static float calculateAngle(const QVector3D& cursor, const QVector3D& anchor);
  void clearOgreOverlay() const;
  void refreshArrowVisual() const;
  void drawArrowVisual(rendering::SceneOverlay* scene) const;
  void hideArrowVisual() const;

  State state_ = State::kPosition;
  float angle_ = 0.f;
  bool arrow_visible_ = false;
  std::optional<QVector3D> arrow_position_;
  std::optional<QVector3D> committed_position_;
  float committed_angle_ = 0.f;
};

}  // namespace tools
}  // namespace autoviz
