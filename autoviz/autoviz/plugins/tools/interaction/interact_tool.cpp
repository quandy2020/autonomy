/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/plugins/tools/interaction/interact_tool.hpp"

#include <functional>

#include <QMenu>
#include <QMatrix4x4>
#include <QMouseEvent>

#include <automsgs/msgs/visualization_msgs/interactive_marker_feedback.pb.h>
#include "autoviz/plugins/displays/display.hpp"
#include "autoviz/plugins/displays/interactive_markers/interactive_marker_registry.hpp"
#include "autoviz/plugins/displays/marker/marker_draw_utils.hpp"
#include "autoviz/rendering/view_controller.hpp"
#include "autoviz/plugins/tools/common/publish_tool_utils.hpp"

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
  if (context != nullptr && context->set_status) {
    context->set_status(
        QStringLiteral("Click objects to interact with them. "
                       "Right-click for context menu."));
  }
}

void InteractTool::updateContext(common::ToolContext* context) {
  Tool::updateContext(context);
  registry_ = context != nullptr ? context->interactive_markers : nullptr;
  display_context_ = context != nullptr ? context->display_context : nullptr;
}

void InteractTool::deactivate() {
  dragging_ = false;
  active_pick_.reset();
  if (registry_ != nullptr) {
    registry_->clearHighlight();
  }
  registry_ = nullptr;
  display_context_ = nullptr;
  feedback_writer_.reset();
  feedback_writer_channel_.clear();
  Tool::deactivate();
}

void InteractTool::updateHighlight(const display::InteractiveMarkerPick* pick,
                                   bool active, bool ctrl_held) {
  if (registry_ == nullptr) {
    return;
  }
  if (pick == nullptr || !pick->hit) {
    registry_->clearHighlight();
    return;
  }
  display::InteractiveMarkerHighlight hl;
  hl.level = active ? display::InteractiveMarkerHighlight::Level::kActive
                    : display::InteractiveMarkerHighlight::Level::kHover;
  hl.kind = display::InteractiveMarkerRegistry::highlightKind(
      pick->interaction_mode, ctrl_held);
  hl.marker_name = pick->marker_name;
  hl.control_name = pick->control_name;
  hl.interaction_mode = pick->interaction_mode;
  registry_->setHighlight(hl);
}

bool InteractTool::ensureFeedbackWriter(const std::string& channel) {
  if (context() == nullptr || context()->autolink_node == nullptr ||
      channel.empty()) {
    feedback_writer_.reset();
    feedback_writer_channel_.clear();
    return false;
  }
  if (feedback_writer_ != nullptr && feedback_writer_channel_ == channel) {
    return true;
  }
  feedback_writer_ =
      context()->autolink_node->CreateWriter<FeedbackMsg>(channel);
  feedback_writer_channel_ =
      feedback_writer_ != nullptr ? channel : std::string{};
  return feedback_writer_ != nullptr;
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
  if (!ensureFeedbackWriter(active_pick_->feedback_channel)) {
    return;
  }

  FeedbackMsg feedback;
  // Prefer marker frame (RViz InteractiveMarker::publishFeedback).
  const std::string frame =
      it->second.marker.header().frame_id().empty()
          ? context()->fixed_frame
          : it->second.marker.header().frame_id();
  FillHeader(feedback.mutable_header(), frame);
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
  feedback_writer_->Write(std::make_shared<FeedbackMsg>(feedback));
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
  publishFeedback(kEventKeepAlive, drag_initial_point_, true);
}

bool InteractTool::resolveDragPoint(int pixel_x, int pixel_y,
                                    QVector3D* drag_point,
                                    bool shift_held) const {
  if (drag_point == nullptr || !active_pick_.has_value() ||
      context() == nullptr || context()->view_controller == nullptr) {
    return false;
  }
  QVector3D origin;
  QVector3D direction;
  if (!context()->view_controller->pickRay(
          pixel_x, pixel_y, context()->viewport_width,
          context()->viewport_height, &origin, &direction)) {
    return false;
  }
  const QMatrix4x4 inv_view =
      context()->view_controller->viewMatrix().inverted();
  const QVector3D view_forward =
      (inv_view * QVector4D(0.f, 0.f, -1.f, 0.f)).toVector3D().normalized();
  return display::InteractiveMarkerRegistry::dragPointFromRay(
      active_pick_->interaction_mode, active_pick_->control_transform, origin,
      direction, view_forward, drag_point, shift_held);
}

bool InteractTool::updateDraggedPose(const QVector3D& drag_point,
                                     bool shift_held, bool ctrl_held) {
  if (!active_pick_.has_value() || registry_ == nullptr) {
    return false;
  }
  const auto pose = display::InteractiveMarkerRegistry::draggedPose(
      drag_initial_pose_, drag_marker_fixed_, active_pick_->control_transform,
      active_pick_->interaction_mode, drag_initial_point_, drag_point,
      shift_held, ctrl_held);
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
    updateHighlight(nullptr, false);
    if (context()->request_redraw) {
      context()->request_redraw();
    }
    return false;
  }

  if (pick.interaction_mode == kInteractionMenu) {
    showMarkerMenu(pick, event);
    return true;
  }

  active_pick_ = pick;
  updateHighlight(&pick, true,
                  event->modifiers().testFlag(Qt::ControlModifier));
  const auto it = registry_->markers().find(pick.marker_name);
  if (it != registry_->markers().end()) {
    drag_initial_pose_ = it->second.marker.pose();
    automsgs::msgs::visualization_msgs::Marker pose_marker;
    pose_marker.mutable_header()->set_frame_id(
        it->second.marker.header().frame_id());
    pose_marker.mutable_pose()->CopyFrom(it->second.marker.pose());
    drag_marker_fixed_ =
        display::markerPoseTransformInFixedFrame(pose_marker, display_context_);
  } else {
    drag_marker_fixed_.setToIdentity();
  }

  QVector3D drag_point = pick.position;
  const bool shift_down = event->modifiers().testFlag(Qt::ShiftModifier);
  if (!resolveDragPoint(event->pos().x(), event->pos().y(), &drag_point,
                        shift_down)) {
    drag_point = pick.position;
  }
  drag_initial_point_ = drag_point;
  last_keep_alive_ = {};

  if (pick.interaction_mode == kInteractionButton) {
    publishFeedback(kEventButtonClick, drag_point, true);
    active_pick_.reset();
    if (context()->set_status) {
      context()->set_status(statusText());
    }
    return true;
  }

  dragging_ = true;
  // RViz InteractionTool::hide_inactive: HandlerManager::enableInteraction is
  // not wired in Autoviz yet; InteractiveMarkerRegistry has no equivalent API.
  (void)propertyValue("hide_inactive", "true");
  publishFeedback(kEventMouseDown, drag_point, true);
  if (context()->request_redraw) {
    context()->request_redraw();
  }
  if (context()->set_status) {
    context()->set_status(statusText());
  }
  return true;
}

bool InteractTool::mouseMoveEvent(QMouseEvent* event) {
  if (context() == nullptr || context()->view_controller == nullptr ||
      registry_ == nullptr || display_context_ == nullptr) {
    return false;
  }
  const bool ctrl_held = event->modifiers().testFlag(Qt::ControlModifier);

  // Hover highlight when idle (RViz InteractionTool).
  if (!dragging_) {
    const display::InteractiveMarkerPick pick = registry_->pickMarker(
        context()->view_controller, context()->viewport_width,
        context()->viewport_height, event->pos().x(), event->pos().y(),
        display_context_);
    updateHighlight(pick.hit ? &pick : nullptr, false, ctrl_held);
    if (context()->request_redraw) {
      context()->request_redraw();
    }
    if (context()->set_status) {
      context()->set_status(statusText());
    }
    return pick.hit;
  }

  if (!active_pick_.has_value()) {
    return false;
  }
  // Refresh translate/rotate cue when Ctrl toggles during 6-DoF drag.
  updateHighlight(&(*active_pick_), true, ctrl_held);

  const bool shift_held = event->modifiers().testFlag(Qt::ShiftModifier);
  QVector3D drag_point;
  if (!resolveDragPoint(event->pos().x(), event->pos().y(), &drag_point,
                        shift_held)) {
    maybeSendKeepAlive();
    return true;
  }
  if (updateDraggedPose(drag_point, shift_held, ctrl_held)) {
    publishFeedback(kEventPoseUpdate, drag_point, true);
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
  QVector3D drag_point;
  bool have_point = false;
  if (context() != nullptr && context()->view_controller != nullptr) {
    const bool shift_held = event->modifiers().testFlag(Qt::ShiftModifier);
    const bool ctrl_held = event->modifiers().testFlag(Qt::ControlModifier);
    have_point = resolveDragPoint(event->pos().x(), event->pos().y(),
                                  &drag_point, shift_held);
    if (have_point) {
      updateDraggedPose(drag_point, shift_held, ctrl_held);
    }
  }
  publishFeedback(kEventPoseUpdate, drag_point, have_point);
  publishFeedback(kEventMouseUp, drag_point, have_point);
  dragging_ = false;
  active_pick_.reset();
  updateHighlight(nullptr, false);
  if (context() != nullptr && context()->request_redraw) {
    context()->request_redraw();
  }
  if (context() != nullptr && context()->set_status) {
    context()->set_status(statusText());
  }
  return true;
}

QString InteractTool::statusText() const {
  if (registry_ != nullptr) {
    const auto& hl = registry_->highlight();
    if (hl.level != display::InteractiveMarkerHighlight::Level::kNone &&
        !hl.marker_name.empty()) {
      QString kind;
      switch (hl.kind) {
        case display::InteractiveMarkerHighlight::Kind::kTranslate:
          kind = QStringLiteral("TRANSLATE");
          break;
        case display::InteractiveMarkerHighlight::Kind::kRotate:
          kind = QStringLiteral("ROTATE");
          break;
        case display::InteractiveMarkerHighlight::Kind::kBoth:
          kind = QStringLiteral("MOVE+ROTATE");
          break;
        default:
          kind = QStringLiteral("INTERACT");
          break;
      }
      const QString phase =
          hl.level == display::InteractiveMarkerHighlight::Level::kActive
              ? QStringLiteral("dragging")
              : QStringLiteral("hover");
      return QStringLiteral("Interact [%1] %2 — %3 / %4")
          .arg(kind, phase, QString::fromStdString(hl.marker_name),
               QString::fromStdString(hl.control_name));
    }
  }
  if (!active_pick_.has_value()) {
    return QStringLiteral("Click objects to interact with them. "
                          "Right-click for context menu. "
                          "Cyan=translate, Amber=rotate. "
                          "6-DoF cube: drag=any direction in view, "
                          "Shift=up/down, Ctrl=rotate.");
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
    return QStringLiteral("%1 · %2")
        .arg(mode)
        .arg(QString::fromStdString(active_pick_->marker_name));
  }
  return QStringLiteral("dragging %1 (%2)")
      .arg(QString::fromStdString(active_pick_->marker_name))
      .arg(mode);
}

}  // namespace tools
}  // namespace autoviz
