/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <chrono>
#include <optional>
#include <string>

#include <QVector3D>

#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include "autoviz/common/tool.hpp"
#include "autoviz/display/interactive_marker_registry.hpp"

class QMenu;
class QMouseEvent;

namespace autoviz {
namespace tools {

class InteractTool : public common::Tool {
 public:
  std::string id() const override { return "Interact"; }
  QString label() const override { return QStringLiteral("Interact"); }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override {
    return {{"client_id", "Client ID", "autoviz"}};
  }

  void activate(common::ToolContext* context) override;
  void updateContext(common::ToolContext* context) override;
  void deactivate() override;

  bool mousePressEvent(QMouseEvent* event) override;
  bool mouseMoveEvent(QMouseEvent* event) override;
  bool mouseReleaseEvent(QMouseEvent* event) override;
  QString statusText() const override;

 private:
  void publishFeedback(uint32_t event_type, const QVector3D& mouse_point,
                       bool mouse_point_valid, uint32_t menu_entry_id = 0);
  bool updateDraggedPose(const QVector3D& ground_point, bool shift_held);
  void showMarkerMenu(const display::InteractiveMarkerPick& pick,
                      const QMouseEvent* event);
  void maybeSendKeepAlive();

  display::InteractiveMarkerRegistry* registry_ = nullptr;
  common::DisplayContext* display_context_ = nullptr;
  std::optional<display::InteractiveMarkerPick> active_pick_;
  automsgs::msgs::geometry_msgs::Pose drag_initial_pose_;
  QVector3D drag_initial_ground_{0.f, 0.f, 0.f};
  QVector3D drag_offset_{0.f, 0.f, 0.f};
  bool dragging_ = false;
  std::chrono::steady_clock::time_point last_keep_alive_{};
};

}  // namespace tools
}  // namespace autoviz
