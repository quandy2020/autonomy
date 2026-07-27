/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/tools/goal_pose_tool.hpp"

#include <QtMath>

#include <QMouseEvent>

#include "autoviz/common/display_context.hpp"
#include "autoviz/display/ogre_overlay_draw.hpp"
#include "autoviz/rendering/scene_overlay.hpp"
#include "autoviz/rendering/view_controller.hpp"

#ifdef AUTOVIZ_USE_OGRE
#include "autoviz/rendering/ogre_scene_host.hpp"
#endif

namespace autoviz {
namespace tools {
namespace {

// rviz_rendering::Arrow: shaft 2.0 + head 0.5
constexpr float kArrowReach = 2.5f;
constexpr float kGroundLift = 0.02f;

QVector3D liftAboveGround(const QVector3D& point) {
  return point + QVector3D(0.f, 0.f, kGroundLift);
}

QVector3D directionFromYaw(float yaw) {
  return QVector3D(std::cos(yaw), std::sin(yaw), 0.f);
}

std::string poseArrowDisplayName(const std::string& tool_id) {
  return "AvizTool/" + tool_id + "/pose_arrow";
}

}  // namespace

void GoalPoseTool::activate(common::ToolContext* context) {
  common::Tool::activate(context);
  state_ = State::kPosition;
  angle_ = 0.f;
  arrow_visible_ = false;
  arrow_position_.reset();
  hideArrowVisual();
  if (context != nullptr && context->set_status) {
    context->set_status(statusText());
  }
}

void GoalPoseTool::clearOgreOverlay() const {
#ifdef AUTOVIZ_USE_OGRE
  if (context() == nullptr || context()->display_context == nullptr ||
      context()->display_context->ogre_scene_host == nullptr) {
    return;
  }
  context()->display_context->ogre_scene_host->clearToolOverlay(toolId());
#endif
}

void GoalPoseTool::hideArrowVisual() const {
#ifdef AUTOVIZ_USE_OGRE
  if (context() != nullptr && context()->sync_ogre_host) {
    context()->sync_ogre_host();
  }
  common::DisplayContext* display_context =
      context() != nullptr ? context()->display_context : nullptr;
  if (display_context != nullptr &&
      display_context->ogre_scene_host != nullptr) {
    display_context->ogre_scene_host->setToolPoseArrow(
        toolId(), QVector3D(), 0.f, arrowColor(), false);
  }
#endif
}

void GoalPoseTool::drawArrowVisual(rendering::SceneOverlay* scene) const {
  QVector3D pos;
  float yaw = 0.f;
  bool visible = false;

  if (state_ == State::kOrientation && arrow_position_.has_value() &&
      arrow_visible_) {
    pos = *arrow_position_;
    yaw = angle_;
    visible = true;
  } else if (committed_position_.has_value()) {
    pos = *committed_position_;
    yaw = committed_angle_;
    visible = true;
  }

  if (!visible) {
    hideArrowVisual();
    return;
  }

  const QVector3D lifted = liftAboveGround(pos);
  const QColor color = arrowColor();
  const QVector3D tip = lifted + directionFromYaw(yaw) * kArrowReach;
  const std::string overlay_name = poseArrowDisplayName(toolId());

  if (context() != nullptr && context()->sync_ogre_host) {
    context()->sync_ogre_host();
  }

  rendering::SceneOverlay* draw_scene =
      scene != nullptr ? scene : (context() != nullptr ? context()->scene_overlay
                                                     : nullptr);
  if (draw_scene != nullptr && context() != nullptr) {
    display::drawArrowOgreOrGl(context()->display_context, *draw_scene,
                               overlay_name, lifted, tip, color, 0.2f, 0.2f,
                               0.35f);
    return;
  }

#ifdef AUTOVIZ_USE_OGRE
  common::DisplayContext* display_context =
      context() != nullptr ? context()->display_context : nullptr;
  if (display_context != nullptr &&
      display_context->ogre_scene_host != nullptr) {
    display_context->ogre_scene_host->setToolPoseArrow(toolId(), lifted, yaw,
                                                       color, true);
  }
#endif
}

void GoalPoseTool::refreshArrowVisual() const {
  drawArrowVisual(context() != nullptr ? context()->scene_overlay : nullptr);
  if (context() != nullptr && context()->request_redraw) {
    context()->request_redraw();
  }
}

void GoalPoseTool::deactivate() {
  state_ = State::kPosition;
  angle_ = 0.f;
  arrow_visible_ = false;
  arrow_position_.reset();
  committed_position_.reset();
  committed_angle_ = 0.f;
  clearOgreOverlay();
  common::Tool::deactivate();
}

bool GoalPoseTool::pickGround(int x, int y, QVector3D* hit) const {
  if (context() == nullptr || context()->view_controller == nullptr ||
      hit == nullptr) {
    return false;
  }
  return context()->view_controller->pickGroundPoint(
      x, y, context()->viewport_width, context()->viewport_height, hit);
}

float GoalPoseTool::calculateAngle(const QVector3D& cursor,
                                   const QVector3D& anchor) {
  // RViz PoseTool::calculateAngle on the fixed-frame XY ground plane.
  return std::atan2(cursor.y() - anchor.y(), cursor.x() - anchor.x());
}

bool GoalPoseTool::mousePressEvent(QMouseEvent* event) {
  if (event->button() != Qt::LeftButton) {
    return true;
  }
  if (state_ != State::kPosition) {
    return true;
  }
  if (context() == nullptr || context()->view_controller == nullptr) {
    return true;
  }

  QVector3D hit;
  if (!pickGround(event->pos().x(), event->pos().y(), &hit)) {
    return true;
  }

  arrow_position_ = hit;
  arrow_visible_ = false;
  angle_ = 0.f;
  state_ = State::kOrientation;
  if (context()->set_status) {
    context()->set_status(statusText());
  }
  hideArrowVisual();
  if (context()->request_redraw) {
    context()->request_redraw();
  }
  return true;
}

bool GoalPoseTool::mouseMoveEvent(QMouseEvent* event) {
  if (state_ != State::kOrientation) {
    return true;
  }
  if ((event->buttons() & Qt::LeftButton) == 0) {
    return true;
  }
  if (!arrow_position_.has_value()) {
    return true;
  }

  QVector3D hit;
  if (!pickGround(event->pos().x(), event->pos().y(), &hit)) {
    return true;
  }

  angle_ = calculateAngle(hit, *arrow_position_);
  arrow_visible_ = true;
  refreshArrowVisual();
  return true;
}

bool GoalPoseTool::mouseReleaseEvent(QMouseEvent* event) {
  if (event->button() != Qt::LeftButton) {
    return true;
  }
  if (state_ != State::kOrientation || !arrow_position_.has_value()) {
    return true;
  }

  onPoseSet(*arrow_position_, angle_);

  committed_position_ = *arrow_position_;
  committed_angle_ = angle_;

  state_ = State::kPosition;
  arrow_visible_ = false;
  arrow_position_.reset();
  angle_ = 0.f;
  refreshArrowVisual();

  if (context() != nullptr && context()->request_redraw) {
    context()->request_redraw();
  }
  if (context() != nullptr && context()->set_status) {
    context()->set_status(statusText());
  }
  return true;
}

void GoalPoseTool::onDraw(rendering::SceneOverlay& scene) {
  drawArrowVisual(&scene);
}

QString GoalPoseTool::statusText() const {
  if (state_ == State::kOrientation) {
    return QStringLiteral(
        "拖动鼠标设定朝向，松开左键发送目标（geometry_msgs/PoseStamped）。");
  }
  return QStringLiteral(
      "在地图上单击设位置，按住左键拖动设朝向，松开发送。快捷键 G。");
}

}  // namespace tools
}  // namespace autoviz
