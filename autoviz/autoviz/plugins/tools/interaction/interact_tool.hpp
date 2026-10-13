/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file interact_tool.hpp
 * @brief Interact tool — drag / menu interactive markers (RViz InteractTool).
 *
 * Picks @ref display::InteractiveMarkerRegistry controls, publishes feedback
 * (MOUSE_DOWN / MOVE / UP / MENU_SELECT / KEEP_ALIVE), and supports shift-drag
 * pose updates. Client ID is configurable via the @c client_id property.
 *
 * @see display::InteractiveMarkerRegistry
 * @see display::InteractiveMarkerPick
 * @see SelectTool
 * @see common::Tool
 */

#pragma once

#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include <QMatrix4x4>
#include <QVector3D>

#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/visualization_msgs/interactive_marker_feedback.pb.h>
#include "autolink/node/writer.hpp"
#include "autoviz/common/tool.hpp"
#include "autoviz/plugins/displays/interactive_markers/interactive_marker_registry.hpp"

class QMenu;
class QMouseEvent;

namespace autoviz {
namespace tools {

/**
 * @class InteractTool
 * @brief Interactive-marker manipulation tool with feedback publishing and
 *        context menus.
 *
 * ## Properties
 *
 * | Key       | Label     | Default |
 * |-----------|-----------|---------|
 * | client_id | Client ID | autoviz |
 *
 * ## Interaction
 *
 * - **Press:** pick marker control; begin drag or open menu on right-click.
 * - **Move:** update dragged pose and publish MOUSE_MOVE feedback.
 * - **Release:** end drag and publish MOUSE_UP.
 * - **Idle:** periodic KEEP_ALIVE while a marker remains active.
 *
 * @note Requires @c ToolContext::interactive_markers and typically
 *       @c display_context for frame / publish helpers.
 */
class InteractTool : public common::Tool {
 public:
  /**
   * @brief Stable tool id for the registry / session config.
   * @return @c "Interact".
   */
  std::string id() const override { return "Interact"; }

  QString label() const override { return QStringLiteral("Interact"); }

  char shortcutKey() const override { return 'i'; }

  std::vector<common::DisplayPropertySpec> propertySpecs() const override {
    return {{"client_id", "Client ID", "autoviz"},
            {"hide_inactive", "Hide Inactive Objects", "true",
             {"true", "false"}}};
  }

  /**
   * @brief Caches registry / display context pointers from @p context.
   * @param context Active tool context.
   */
  void activate(common::ToolContext* context) override;

  /**
   * @brief Refreshes registry / context pointers without resetting drag state.
   * @param context Updated tool context (e.g. after viewport switch).
   */
  void updateContext(common::ToolContext* context) override;

  /**
   * @brief Ends any active drag and clears cached pointers.
   */
  void deactivate() override;

  /**
   * @brief Begins marker drag or shows the marker context menu.
   *
   * @param event Mouse press in viewport coordinates.
   * @return @c true if an interactive marker handled the press.
   */
  bool mousePressEvent(QMouseEvent* event) override;

  /**
   * @brief Updates drag pose and publishes MOUSE_MOVE feedback.
   *
   * @param event Mouse move in viewport coordinates.
   * @return @c true while a drag is active.
   */
  bool mouseMoveEvent(QMouseEvent* event) override;

  /**
   * @brief Ends drag and publishes MOUSE_UP feedback.
   *
   * @param event Mouse release in viewport coordinates.
   * @return @c true if a drag was completed.
   */
  bool mouseReleaseEvent(QMouseEvent* event) override;

  /**
   * @brief Status text naming the active marker / control when dragging.
   * @return Short interaction summary, or empty when idle.
   */
  QString statusText() const override;

 private:
  /**
   * @brief Publishes InteractiveMarkerFeedback for the active pick.
   *
   * @param event_type Feedback event enum (MOUSE_DOWN, MOVE, UP, …).
   * @param mouse_point World mouse / ground point associated with the event.
   * @param mouse_point_valid Whether @p mouse_point is meaningful.
   * @param menu_entry_id Menu entry id for MENU_SELECT (default @c 0).
   */
  void publishFeedback(uint32_t event_type, const QVector3D& mouse_point,
                       bool mouse_point_valid, uint32_t menu_entry_id = 0);

  /**
   * @brief Applies ray–axis / ray–plane drag to the active marker pose.
   *
   * @param drag_point Current 3D drag sample (fixed frame).
   * @param shift_held When @c true, may constrain / alter drag semantics.
   * @return @c true if the pose was updated.
   */
  bool updateDraggedPose(const QVector3D& drag_point, bool shift_held,
                         bool ctrl_held);

  /**
   * @brief Resolves a fixed-frame drag sample under the cursor (RViz-style).
   */
  bool resolveDragPoint(int pixel_x, int pixel_y, QVector3D* drag_point,
                        bool shift_held = false) const;

  /** Caches Autolink writer for the active feedback channel. */
  bool ensureFeedbackWriter(const std::string& channel);

  /**
   * @brief Shows a Qt context menu for marker menu entries.
   *
   * @param pick Marker / control that was right-clicked.
   * @param event Mouse event used for popup position.
   */
  void showMarkerMenu(const display::InteractiveMarkerPick& pick,
                      const QMouseEvent* event);

  /**
   * @brief Sends KEEP_ALIVE feedback if the keep-alive interval has elapsed.
   */
  void maybeSendKeepAlive();

  /**
   * @brief Updates registry hover/active highlight for translate vs rotate.
   */
  void updateHighlight(const display::InteractiveMarkerPick* pick, bool active,
                       bool ctrl_held = false);

  /** Non-owning interactive marker registry from ToolContext. */
  display::InteractiveMarkerRegistry* registry_ = nullptr;

  /** Non-owning display context for frame / publish helpers. */
  common::DisplayContext* display_context_ = nullptr;

  /** Currently picked marker control, if any. */
  std::optional<display::InteractiveMarkerPick> active_pick_;

  /** Marker pose at drag start (for delta computation). */
  automsgs::msgs::geometry_msgs::Pose drag_initial_pose_;

  /** Marker pose matrix in fixed frame at drag start. */
  QMatrix4x4 drag_marker_fixed_;

  /** 3D drag sample at mouse-down (fixed frame). */
  QVector3D drag_initial_point_{0.f, 0.f, 0.f};

  /** @c true while a marker drag is in progress. */
  bool dragging_ = false;

  /** Timestamp of the last KEEP_ALIVE feedback. */
  std::chrono::steady_clock::time_point last_keep_alive_{};

  using FeedbackMsg =
      automsgs::msgs::visualization_msgs::InteractiveMarkerFeedback;
  std::shared_ptr<::autolink::Writer<FeedbackMsg>> feedback_writer_;
  std::string feedback_writer_channel_;
};

}  // namespace tools
}  // namespace autoviz
