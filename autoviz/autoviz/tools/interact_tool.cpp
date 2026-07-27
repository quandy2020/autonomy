/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/tools/interact_tool.hpp"

#include <functional>

#include <QMenu>
#include <QMouseEvent>

#include <automsgs/msgs/visualization_msgs/interactive_marker_feedback.pb.h>
#include "autoviz/display/display.hpp"
#include "autoviz/display/interactive_marker_registry.hpp"
#include "autoviz/rendering/view_controller.hpp"
#include "autoviz/tools/publish_tool_utils.hpp"

namespace autoviz {
namespace tools {
namespace {

constexpr uint32_t kEventKeepAlive = 0;
constexpr uint32_t kEventPoseUpdate = 1;
constexpr uint32_t kEventMenuSelect = 2;
constexpr uint32_t kEventButtonClick = 3;
constexpr uint32_t kEventMouseDown = 4;
constexpr uint32_t kEventMouseUp = 5;
constexpr uint32_t kInteractionButton = 2;
constexpr uint32_t kInteractionMenu = 1;
constexpr auto kKeepAliveInterval = std::chrono::milliseconds(200);

void PopulateMenu(QMenu* menu,
                  const google::protobuf::RepeatedPtrField<
                      automsgs::msgs::visualization_msgs::MenuEntry>&
                      entries,
                  uint32_t parent_id,
                  const std::function<void(uint32_t)>& on_select) {
  for (const auto& entry : entries) {
    if (entry.parent_id() != parent_id) {
      continue;
    }
    bool has_children = false;
    for (const auto& child : entries) {
      if (child.parent_id() == entry.id()) {
        has_children = true;
        break;
      }
    }
    if (has_children) {
      auto* submenu = menu->addMenu(QString::fromStdString(entry.title()));
      PopulateMenu(submenu, entries, entry.id(), on_select);
    } else {
      const uint32_t entry_id = entry.id();
      QObject::connect(menu->addAction(QString::fromStdString(entry.title())),
                       &QAction::triggered,
                       [on_select, entry_id]() { on_select(entry_id); });
    }
  }
}

}  // namespace

void InteractTool::activate(common::ToolContext* context) {
  updateContext(context);
}

void InteractTool::updateContext(common::ToolContext* context) {
  Tool::updateContext(context);
  registry_ = context != nullptr ? context->interactive_markers : nullptr;
  display_context_ = context != nullptr ? context->display_context : nullptr;
}

void InteractTool::deactivate() {
  dragging_ = false;
  active_pick_.reset();
  registry_ = nullptr;
  display_context_ = nullptr;
  Tool::deactivate();
}

void InteractTool::publishFeedback(uint32_t event_type,
                                     const QVector3D& mouse_point,
                                     bool mouse_point_valid,
                                     uint32_t menu_entry_id) {
  if (!active_pick_.has_value() || context() == nullptr ||
      context()->autolink_node == nullptr ||
      active_pick_->feedback_channel.empty() || registry_ == nullptr) {
    return;
  }
  const auto it = registry_->markers().find(active_pick_->marker_name);
  if (it == registry_->markers().end()) {
    return;
  }

  automsgs::msgs::visualization_msgs::InteractiveMarkerFeedback feedback;
  FillHeader(feedback.mutable_header(), context()->fixed_frame);
  feedback.set_client_id(propertyValue("client_id", "autoviz"));
  feedback.set_marker_name(active_pick_->marker_name);
  feedback.set_control_name(active_pick_->control_name);
  feedback.set_event_type(
      static_cast<automsgs::msgs::visualization_msgs::
                      InteractiveMarkerFeedback_EventType>(event_type));
  feedback.mutable_pose()->CopyFrom(it->second.marker.pose());
  feedback.set_menu_entry_id(menu_entry_id);
  if (mouse_point_valid) {
    feedback.mutable_mouse_point()->set_x(mouse_point.x());
    feedback.mutable_mouse_point()->set_y(mouse_point.y());
    feedback.mutable_mouse_point()->set_z(mouse_point.z());
    feedback.set_mouse_point_valid(true);
  }
  PublishMessage(context()->autolink_node, active_pick_->feedback_channel,
                 feedback);
}

void InteractTool::maybeSendKeepAlive() {
  if (!dragging_) {
    return;
  }
  const auto now = std::chrono::steady_clock::now();
  if (last_keep_alive_.time_since_epoch().count() != 0 &&
      now - last_keep_alive_ < kKeepAliveInterval) {
    return;
  }
  last_keep_alive_ = now;
  publishFeedback(kEventKeepAlive, drag_initial_ground_, true);
}

bool InteractTool::updateDraggedPose(const QVector3D& ground_point,
                                     bool shift_held) {
  if (!active_pick_.has_value() || registry_ == nullptr) {
    return false;
  }
  const auto pose = display::InteractiveMarkerRegistry::draggedPose(
      drag_initial_pose_, active_pick_->control_transform,
      active_pick_->interaction_mode, drag_initial_ground_, ground_point,
      shift_held);
  return registry_->updatePose(active_pick_->marker_name, pose);
}

void InteractTool::showMarkerMenu(const display::InteractiveMarkerPick& pick,
                                  const QMouseEvent* event) {
  if (registry_ == nullptr) {
    return;
  }
  const auto it = registry_->markers().find(pick.marker_name);
  if (it == registry_->markers().end() ||
      it->second.marker.menu_entries().empty()) {
    return;
  }

  active_pick_ = pick;
  QMenu menu;
  PopulateMenu(
      &menu, it->second.marker.menu_entries(), 0,
      [this](uint32_t entry_id) {
        publishFeedback(kEventMenuSelect, active_pick_->position, true,
                        entry_id);
        if (context() != nullptr && context()->set_status) {
          context()->set_status(
              QStringLiteral("Interact: menu entry %1 on %2")
                  .arg(entry_id)
                  .arg(QString::fromStdString(active_pick_->marker_name)));
        }
      });
  menu.exec(event->globalPosition().toPoint());
  active_pick_.reset();
}

bool InteractTool::mousePressEvent(QMouseEvent* event) {
  if (context() == nullptr || context()->view_controller == nullptr ||
      registry_ == nullptr || display_context_ == nullptr) {
    return false;
  }

  const display::InteractiveMarkerPick pick = registry_->pickMarker(
      context()->view_controller, context()->viewport_width,
      context()->viewport_height, event->pos().x(), event->pos().y(),
      display_context_);

  if (event->button() == Qt::RightButton) {
    if (pick.hit) {
      showMarkerMenu(pick, event);
      return true;
    }
    return false;
  }

  if (event->button() != Qt::LeftButton || !pick.hit) {
    active_pick_.reset();
    dragging_ = false;
    return false;
  }

  if (pick.interaction_mode == kInteractionMenu) {
    showMarkerMenu(pick, event);
    return true;
  }

  QVector3D ground;
  if (!context()->view_controller->pickGroundPoint(
          event->pos().x(), event->pos().y(), context()->viewport_width,
          context()->viewport_height, &ground)) {
    ground = pick.position;
  }

  active_pick_ = pick;
  const auto it = registry_->markers().find(pick.marker_name);
  if (it != registry_->markers().end()) {
    drag_initial_pose_ = it->second.marker.pose();
  }
  drag_initial_ground_ = ground;
  drag_offset_ = pick.position - ground;
  last_keep_alive_ = {};

  if (pick.interaction_mode == kInteractionButton) {
    publishFeedback(kEventButtonClick, ground, true);
    active_pick_.reset();
    if (context()->set_status) {
      context()->set_status(statusText());
    }
    return true;
  }

  dragging_ = true;
  publishFeedback(kEventMouseDown, ground, true);
  if (context()->request_redraw) {
    context()->request_redraw();
  }
  if (context()->set_status) {
    context()->set_status(statusText());
  }
  return true;
}

bool InteractTool::mouseMoveEvent(QMouseEvent* event) {
  if (!dragging_ || !active_pick_.has_value() || context() == nullptr ||
      context()->view_controller == nullptr) {
    return false;
  }
  QVector3D ground;
  if (!context()->view_controller->pickGroundPoint(
          event->pos().x(), event->pos().y(), context()->viewport_width,
          context()->viewport_height, &ground)) {
    maybeSendKeepAlive();
    return true;
  }
  const bool shift_held = event->modifiers().testFlag(Qt::ShiftModifier);
  if (updateDraggedPose(ground, shift_held)) {
    publishFeedback(kEventPoseUpdate, ground, true);
    maybeSendKeepAlive();
    if (context()->request_redraw) {
      context()->request_redraw();
    }
    if (context()->set_status) {
      context()->set_status(statusText());
    }
  }
  return true;
}

bool InteractTool::mouseReleaseEvent(QMouseEvent* event) {
  if (event->button() != Qt::LeftButton || !dragging_) {
    return false;
  }
  QVector3D ground;
  bool have_ground = false;
  if (context() != nullptr && context()->view_controller != nullptr) {
    have_ground = context()->view_controller->pickGroundPoint(
        event->pos().x(), event->pos().y(), context()->viewport_width,
        context()->viewport_height, &ground);
    if (have_ground) {
      const bool shift_held = event->modifiers().testFlag(Qt::ShiftModifier);
      updateDraggedPose(ground, shift_held);
    }
  }
  publishFeedback(kEventPoseUpdate, ground, have_ground);
  publishFeedback(kEventMouseUp, ground, have_ground);
  dragging_ = false;
  active_pick_.reset();
  if (context() != nullptr && context()->request_redraw) {
    context()->request_redraw();
  }
  if (context() != nullptr && context()->set_status) {
    context()->set_status(statusText());
  }
  return true;
}

QString InteractTool::statusText() const {
  if (!active_pick_.has_value()) {
    return QStringLiteral(
        "Interact: Left rotate · Middle/Shift+Left pan · Right/Wheel zoom · "
        "marker drag/menu when hit");
  }
  QString mode;
  switch (active_pick_->interaction_mode) {
    case 3:
      mode = QStringLiteral("move axis");
      break;
    case 4:
      mode = QStringLiteral("move plane");
      break;
    case 5:
      mode = QStringLiteral("rotate axis");
      break;
    case 6:
      mode = QStringLiteral("move+rotate");
      break;
    case 7:
      mode = QStringLiteral("move 3D");
      break;
    case 8:
      mode = QStringLiteral("rotate 3D");
      break;
    case 9:
      mode = QStringLiteral("6-DOF");
      break;
    default:
      mode = QStringLiteral("control");
      break;
  }
  if (!dragging_) {
    return QStringLiteral("Interact: %1 on %2")
        .arg(mode)
        .arg(QString::fromStdString(active_pick_->marker_name));
  }
  return QStringLiteral("Interact: dragging %1 (%2)")
      .arg(QString::fromStdString(active_pick_->marker_name))
      .arg(mode);
}

}  // namespace tools
}  // namespace autoviz
