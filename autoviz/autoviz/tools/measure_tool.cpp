/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/tools/measure_tool.hpp"

#include <QColor>
#include <QMouseEvent>

#include "autoviz/common/display_context.hpp"
#include "autoviz/rendering/pick_utils.hpp"
#include "autoviz/rendering/scene_overlay.hpp"
#include "autoviz/rendering/view_controller.hpp"

#ifdef AUTOVIZ_USE_OGRE
#include "autoviz/rendering/ogre_scene_host.hpp"
#endif

namespace autoviz {
namespace tools {
namespace {

/** RViz MeasureTool default: Qt::darkYellow */
constexpr QColor kLineColor(184, 134, 11);
/** Lift above reference grid (z=0) to avoid depth fighting with grid lines. */
constexpr float kLineLift = 0.02f;

QVector3D liftAboveGrid(const QVector3D& point) {
  return point + QVector3D(0.f, 0.f, kLineLift);
}

}  // namespace

void MeasureTool::clearOgreOverlay() const {
#ifdef AUTOVIZ_USE_OGRE
  if (context() == nullptr || context()->display_context == nullptr ||
      context()->display_context->ogre_scene_host == nullptr) {
    return;
  }
  context()->display_context->ogre_scene_host->clearToolOverlay(kToolOgreId);
#endif
}

void MeasureTool::updateOgreLineVisual(const QVector3D& start,
                                       const QVector3D& end) const {
#ifdef AUTOVIZ_USE_OGRE
  if (context() != nullptr && context()->sync_ogre_host) {
    context()->sync_ogre_host();
  }
  common::DisplayContext* display_context =
      context() != nullptr ? context()->display_context : nullptr;
  if (display_context == nullptr ||
      display_context->ogre_scene_host == nullptr) {
    return;
  }
  const QVector3D draw_start = liftAboveGrid(start);
  const QVector3D draw_end = liftAboveGrid(end);
  display_context->ogre_scene_host->setToolLineSegment(
      kToolOgreId, draw_start, draw_end, kLineColor);
#endif
}

void MeasureTool::resetMeasurement() {
  line_started_ = false;
  start_point_.reset();
  end_point_.reset();
  hover_point_.reset();
  clearOgreOverlay();
}

void MeasureTool::deactivate() {
  resetMeasurement();
  common::Tool::deactivate();
}

bool MeasureTool::pickPoint(int x, int y, QVector3D* hit) const {
  if (hit == nullptr || context() == nullptr ||
      context()->view_controller == nullptr) {
    return false;
  }
  const rendering::PickResult pick =
      rendering::pickAtToolContext(*context(), x, y);
  if (pick.hit) {
    *hit = pick.position();
    return true;
  }
  return context()->view_controller->pickGroundPoint(
      x, y, context()->viewport_width, context()->viewport_height, hit);
}

std::optional<QVector3D> MeasureTool::endPreview() const {
  if (line_started_ && hover_point_.has_value()) {
    return hover_point_;
  }
  if (end_point_.has_value()) {
    return end_point_;
  }
  return std::nullopt;
}

float MeasureTool::currentLength() const {
  if (!start_point_.has_value()) {
    return -1.f;
  }
  const std::optional<QVector3D> end = endPreview();
  if (!end.has_value()) {
    return -1.f;
  }
  return (*start_point_ - *end).length();
}

void MeasureTool::updateStatus() const {
  if (context() == nullptr || !context()->set_status) {
    return;
  }
  context()->set_status(statusText());
}

void MeasureTool::refreshLineVisual() const {
  if (!start_point_.has_value()) {
    clearOgreOverlay();
    return;
  }
  const std::optional<QVector3D> end = endPreview();
  if (!end.has_value()) {
    clearOgreOverlay();
    return;
  }

  updateOgreLineVisual(*start_point_, *end);

  if (context() != nullptr && context()->request_redraw) {
    context()->request_redraw();
  }
}

bool MeasureTool::mouseMoveEvent(QMouseEvent* event) {
  if (context() == nullptr || context()->view_controller == nullptr) {
    return true;
  }
  if (!line_started_ || !start_point_.has_value()) {
    return true;
  }

  QVector3D hit;
  if (!pickPoint(event->pos().x(), event->pos().y(), &hit)) {
    return true;
  }

  hover_point_ = hit;
  refreshLineVisual();
  updateStatus();
  if (context()->request_redraw) {
    context()->request_redraw();
  }
  return true;
}

bool MeasureTool::mouseReleaseEvent(QMouseEvent* event) {
  if (event->button() == Qt::RightButton) {
    resetMeasurement();
    updateStatus();
    if (context() != nullptr && context()->request_redraw) {
      context()->request_redraw();
    }
    return true;
  }
  if (event->button() != Qt::LeftButton) {
    return true;
  }
  if (context() == nullptr || context()->view_controller == nullptr) {
    updateStatus();
    return true;
  }

  QVector3D hit;
  if (!pickPoint(event->pos().x(), event->pos().y(), &hit)) {
    if (context()->set_status) {
      context()->set_status(
          QStringLiteral("Measure: click on rendered geometry "
                           "(right-click to reset)"));
    }
    return true;
  }

  if (line_started_) {
    end_point_ = hit;
    hover_point_.reset();
    line_started_ = false;
  } else {
    start_point_ = hit;
    end_point_.reset();
    hover_point_ = hit;
    line_started_ = true;
  }

  refreshLineVisual();
  updateStatus();
  if (context()->request_redraw) {
    context()->request_redraw();
  }
  return true;
}

void MeasureTool::onDraw(rendering::SceneOverlay& scene) {
  if (!start_point_.has_value()) {
    return;
  }
  const std::optional<QVector3D> end = endPreview();
  if (!end.has_value()) {
    return;
  }

  updateOgreLineVisual(*start_point_, *end);
  scene.addLine(liftAboveGrid(*start_point_), liftAboveGrid(*end), kLineColor);
}

QString MeasureTool::statusText() const {
  QString status;
  const float length = currentLength();
  if (length >= 0.f) {
    status = QStringLiteral("Length: %1 m").arg(length, 0, 'f', 3);
  }

  QString hint;
  if (!start_point_.has_value()) {
    hint = QStringLiteral(
        "Click on two points to measure their distance. Right-click to reset.");
  } else if (line_started_) {
    hint = QStringLiteral("Click second point. Right-click to reset.");
  } else {
    hint = QStringLiteral(
        "Click to start a new measurement. Right-click to reset.");
  }

  if (status.isEmpty()) {
    return QStringLiteral("Measure: %1").arg(hint);
  }
  return QStringLiteral("Measure: %1 | %2").arg(status, hint);
}

}  // namespace tools
}  // namespace autoviz
