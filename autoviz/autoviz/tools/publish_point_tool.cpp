/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/tools/publish_point_tool.hpp"

#include <QColor>
#include <QMouseEvent>

#include <automsgs/msgs/geometry_msgs/point_stamped.pb.h>
#include "autoviz/rendering/pick_utils.hpp"
#include "autoviz/rendering/scene_overlay.hpp"
#include "autoviz/rendering/view_controller.hpp"
#include "autoviz/tools/publish_tool_utils.hpp"
#include "autoviz/ui/icon_loader.hpp"

namespace autoviz {
namespace tools {

void PublishPointTool::activate(common::ToolContext* context) {
  common::Tool::activate(context);
  hit_cursor_ = IconLoader::toolCursor(QStringLiteral("PublishPoint"));
  std_cursor_ = IconLoader::defaultCursor();
  setCursor(hit_cursor_);
}

void PublishPointTool::deactivate() {
  hover_point_.reset();
  common::Tool::deactivate();
}

bool PublishPointTool::pickPoint(int x, int y, QVector3D* hit) const {
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

void PublishPointTool::updateStatusFromPoint(const QVector3D& point,
                                             const QString& prefix) const {
  if (context() == nullptr || !context()->set_status) {
    return;
  }
  context()->set_status(
      QStringLiteral("%1 (%2, %3, %4) → %5")
          .arg(prefix)
          .arg(point.x(), 0, 'f', 3)
          .arg(point.y(), 0, 'f', 3)
          .arg(point.z(), 0, 'f', 3)
          .arg(QString::fromStdString(publishChannel())));
}

bool PublishPointTool::mouseMoveEvent(QMouseEvent* event) {
  if (context() == nullptr || context()->view_controller == nullptr) {
    return true;
  }
  QVector3D hit;
  if (!pickPoint(event->pos().x(), event->pos().y(), &hit)) {
    hover_point_.reset();
    setCursor(std_cursor_);
    if (context()->set_status) {
      context()->set_status(statusText());
    }
    return true;
  }
  hover_point_ = hit;
  setCursor(hit_cursor_);
  updateStatusFromPoint(hit, QStringLiteral("Publish Point:"));
  return true;
}

bool PublishPointTool::mousePressEvent(QMouseEvent* event) {
  if (event->button() != Qt::LeftButton) {
    return true;
  }
  if (context() == nullptr || context()->view_controller == nullptr) {
    if (context() != nullptr && context()->set_status) {
      context()->set_status(
          QStringLiteral("Publish Point: viewport not ready"));
    }
    return true;
  }
  if (context()->autolink_node == nullptr) {
    if (context()->set_status) {
      context()->set_status(
          QStringLiteral("Publish Point: Autolink node unavailable"));
    }
    return true;
  }
  QVector3D hit;
  if (!pickPoint(event->pos().x(), event->pos().y(), &hit)) {
    if (context()->set_status) {
      context()->set_status(
          QStringLiteral("Publish Point: click geometry or the ground plane"));
    }
    return true;
  }
  last_point_ = hit;
  hover_point_ = hit;
  automsgs::msgs::geometry_msgs::PointStamped point;
  FillHeader(point.mutable_header(), context()->fixed_frame);
  point.mutable_point()->set_x(hit.x());
  point.mutable_point()->set_y(hit.y());
  point.mutable_point()->set_z(hit.z());
  const std::string channel = publishChannel();
  if (!PublishMessage(context()->autolink_node, channel, point)) {
    if (context()->set_status) {
      context()->set_status(
          QStringLiteral("Publish Point: failed to publish on %1")
              .arg(QString::fromStdString(channel)));
    }
    return true;
  }
  if (context()->request_redraw) {
    context()->request_redraw();
  }
  updateStatusFromPoint(hit, QStringLiteral("Publish Point: published"));
  return true;
}

void PublishPointTool::onDraw(rendering::SceneOverlay& scene) {
  const std::optional<QVector3D>& point =
      hover_point_.has_value() ? hover_point_ : last_point_;
  if (!point.has_value()) {
    return;
  }
  const QColor color = hover_point_.has_value() ? QColor(255, 180, 80)
                                                : QColor(255, 80, 80);
  scene.addPoint(*point, color);
  const float s = 0.15f;
  scene.addLine(*point - QVector3D(s, 0.f, 0.f), *point + QVector3D(s, 0.f, 0.f),
                color);
  scene.addLine(*point - QVector3D(0.f, s, 0.f), *point + QVector3D(0.f, s, 0.f),
                color);
  scene.addLine(*point - QVector3D(0.f, 0.f, s), *point + QVector3D(0.f, 0.f, s),
                color);
}

QString PublishPointTool::statusText() const {
  const std::optional<QVector3D>& point =
      hover_point_.has_value() ? hover_point_ : last_point_;
  if (point.has_value()) {
    return QStringLiteral("Publish Point: (%1, %2, %3) → %4")
        .arg(point->x(), 0, 'f', 3)
        .arg(point->y(), 0, 'f', 3)
        .arg(point->z(), 0, 'f', 3)
        .arg(QString::fromStdString(publishChannel()));
  }
  return QStringLiteral("Publish Point: click to publish → %1")
      .arg(QString::fromStdString(publishChannel()));
}

}  // namespace tools
}  // namespace autoviz
