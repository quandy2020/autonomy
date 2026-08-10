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
constexpr const char* kDefaultViewportKey = "_default";

QVector3D liftAboveGrid(const QVector3D& point) {
  return point + QVector3D(0.f, 0.f, kLineLift);
}

std::string OgreOverlayId(const std::string& viewport_key) {
  return std::string("Measure:") +
         (viewport_key.empty() ? kDefaultViewportKey : viewport_key);
}

}  // namespace

std::string MeasureTool::currentViewportKey() const {
  if (context() != nullptr && !context()->viewport_key.empty()) {
    return context()->viewport_key;
  }
  return kDefaultViewportKey;
}

MeasureTool::Session& MeasureTool::sessionFor(const std::string& key) {
  return sessions_[key.empty() ? kDefaultViewportKey : key];
}

const MeasureTool::Session* MeasureTool::findSession(
    const std::string& key) const {
  const auto it = sessions_.find(key.empty() ? kDefaultViewportKey : key);
  return it == sessions_.end() ? nullptr : &it->second;
}

void MeasureTool::clearViewportSession(const std::string& viewport_key) {
  const std::string key =
      viewport_key.empty() ? kDefaultViewportKey : viewport_key;
  clearOgreOverlay(key);
  sessions_.erase(key);
}

void MeasureTool::clearOgreOverlay(const std::string& key) const {
#ifdef AUTOVIZ_USE_OGRE
  if (context() == nullptr || context()->display_context == nullptr ||
      context()->display_context->ogre_scene_host == nullptr) {
    return;
  }
  context()->display_context->ogre_scene_host->clearToolOverlay(
      OgreOverlayId(key));
#else
  (void)key;
#endif
}

void MeasureTool::updateOgreLineVisual(const std::string& key,
                                       const QVector3D& start,
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
      OgreOverlayId(key), draw_start, draw_end, kLineColor);
#else
  (void)key;
  (void)start;
  (void)end;
#endif
}

void MeasureTool::resetMeasurement(const std::string& key) {
  clearOgreOverlay(key);
  sessions_.erase(key.empty() ? kDefaultViewportKey : key);
}

void MeasureTool::deactivate() {
  // Keep per-viewport sessions when the global active tool changes (split
  // panels). Sessions are cleared by clearViewportSession / right-click.
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
    *hit = pick.position;
    return true;
  }
  return context()->view_controller->pickGroundPoint(
      x, y, context()->viewport_width, context()->viewport_height, hit);
}

std::optional<QVector3D> MeasureTool::endPreview(const Session& session) const {
  if (session.line_started && session.hover_point.has_value()) {
    return session.hover_point;
  }
  if (session.end_point.has_value()) {
    return session.end_point;
  }
  return std::nullopt;
}

float MeasureTool::currentLength(const Session& session) const {
  if (!session.start_point.has_value()) {
    return -1.f;
  }
  const std::optional<QVector3D> end = endPreview(session);
  if (!end.has_value()) {
    return -1.f;
  }
  return (*session.start_point - *end).length();
}

void MeasureTool::updateStatus() const {
  if (context() == nullptr || !context()->set_status) {
    return;
  }
  context()->set_status(statusText());
}

void MeasureTool::refreshLineVisual(const std::string& key) const {
  const Session* session = findSession(key);
  if (session == nullptr || !session->start_point.has_value()) {
    clearOgreOverlay(key);
    return;
  }
  const std::optional<QVector3D> end = endPreview(*session);
  if (!end.has_value()) {
    clearOgreOverlay(key);
    return;
  }

  updateOgreLineVisual(key, *session->start_point, *end);

  if (context() != nullptr && context()->request_redraw) {
    context()->request_redraw();
  }
}

bool MeasureTool::mouseMoveEvent(QMouseEvent* event) {
  if (context() == nullptr || context()->view_controller == nullptr) {
    return true;
  }
  const std::string key = currentViewportKey();
  Session& session = sessionFor(key);
  if (!session.line_started || !session.start_point.has_value()) {
    return true;
  }

  QVector3D hit;
  if (!pickPoint(event->pos().x(), event->pos().y(), &hit)) {
    return true;
  }

  session.hover_point = hit;
  refreshLineVisual(key);
  updateStatus();
  if (context()->request_redraw) {
    context()->request_redraw();
  }
  return true;
}

bool MeasureTool::mouseReleaseEvent(QMouseEvent* event) {
  const std::string key = currentViewportKey();
  if (event->button() == Qt::RightButton) {
    resetMeasurement(key);
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

  Session& session = sessionFor(key);
  if (session.line_started) {
    session.end_point = hit;
    session.hover_point.reset();
    session.line_started = false;
  } else {
    session.start_point = hit;
    session.end_point.reset();
    session.hover_point = hit;
    session.line_started = true;
  }

  refreshLineVisual(key);
  updateStatus();
  if (context()->request_redraw) {
    context()->request_redraw();
  }
  return true;
}

void MeasureTool::drawSession(rendering::SceneOverlay& scene,
                              const std::string& key,
                              const Session& session) const {
  (void)key;
  if (!session.start_point.has_value()) {
    return;
  }
  const std::optional<QVector3D> end = endPreview(session);
  if (!end.has_value()) {
    return;
  }
  // GL path only: Ogre lines are owned by each window's OgreSceneHost via
  // refreshLineVisual (mouse). Do not re-upload here — sync_ogre_host points
  // at the active dock and would leak Measure into the wrong Split panel.
  scene.addLine(liftAboveGrid(*session.start_point), liftAboveGrid(*end),
                kLineColor);
}

void MeasureTool::onDraw(rendering::SceneOverlay& scene) {
  // Prefer the context viewport (per-panel paint). Fall back to active session.
  const std::string key = currentViewportKey();
  if (const Session* session = findSession(key)) {
    drawSession(scene, key, *session);
  }
}

QString MeasureTool::statusText() const {
  const std::string key = currentViewportKey();
  const Session* session = findSession(key);
  QString status;
  if (session != nullptr) {
    const float length = currentLength(*session);
    if (length >= 0.f) {
      status = QStringLiteral("Length: %1 m").arg(length, 0, 'f', 3);
    }
  }

  QString hint;
  if (session == nullptr || !session->start_point.has_value()) {
    hint = QStringLiteral(
        "Click on two points to measure their distance. Right-click to reset.");
  } else if (session->line_started) {
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
