/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_default_plugins InteractiveMarker
 * (Willow Garage / Open Source Robotics Foundation).
 *****************************************************************************/

/**
 * @file interactive_marker.hpp
 * @brief One InteractiveMarker instance (controls, menu, pose, draw).
 *
 * Mirrors RViz @c InteractiveMarker using Autoviz types (@c QVector3D /
 * @c SceneOverlay / protobuf messages). Feedback is emitted via Qt signals for
 * @ref InteractiveMarkerDisplay to publish on Autolink.
 *
 * @see InteractiveMarkerControl
 * @see InteractiveMarkerDisplay
 */

#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <QMenu>
#include <QObject>
#include <QQuaternion>
#include <QString>
#include <QVector3D>

#include <automsgs/msgs/visualization_msgs/interactive_marker.pb.h>
#include <automsgs/msgs/visualization_msgs/interactive_marker_feedback.pb.h>
#include <automsgs/msgs/visualization_msgs/interactive_marker_pose.pb.h>
#include <automsgs/msgs/visualization_msgs/menu_entry.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/plugins/displays/interactive_markers/interactive_marker_control.hpp"

namespace autoviz {

namespace common {
class DisplayContext;
}

namespace rendering {
class SceneOverlay;
}

namespace display {

/**
 * @class InteractiveMarker
 * @brief Scene-level interactive marker (RViz InteractiveMarker analogue).
 */
class InteractiveMarker : public QObject {
  Q_OBJECT

 public:
  using SharedPtr = std::shared_ptr<InteractiveMarker>;

  /**
   * @brief Constructs an empty marker bound to @p context.
   */
  explicit InteractiveMarker(common::DisplayContext* context);

  ~InteractiveMarker() override = default;

  /**
   * @brief Resets contents from a full InteractiveMarker message.
   * @return @c true on success.
   */
  bool processMessage(
      const automsgs::msgs::visualization_msgs::InteractiveMarker& message);

  /**
   * @brief Applies a pose-only update.
   */
  void processMessage(
      const automsgs::msgs::visualization_msgs::InteractiveMarkerPose&
          message);

  /** Per-frame update (TF-locked reference + controls). */
  void update();

  void setPose(const QVector3D& position, const QQuaternion& orientation,
               const std::string& control_name);
  void translate(const QVector3D& delta_position,
                 const std::string& control_name);
  void rotate(const QQuaternion& delta_orientation,
              const std::string& control_name);

  void requestPoseUpdate(const QVector3D& position,
                         const QQuaternion& orientation);
  void startDragging();
  void stopDragging();

  const QVector3D& getPosition() const { return position_; }
  const QQuaternion& getOrientation() const { return orientation_; }
  float getSize() const { return scale_; }
  const std::string& getReferenceFrame() const { return reference_frame_; }
  const std::string& getName() const { return name_; }

  void setShowDescription(bool show);
  void setShowAxes(bool show);
  void setShowVisualAids(bool show);

  /**
   * @brief Clears hover/active highlights on every control.
   */
  void clearControlHighlights();

  /**
   * @brief Sets RViz-style highlight on one control (by name).
   *
   * @param control_name Control name key.
   * @param highlight Intensity enum.
   * @param kind Translate / rotate visual cue (for draw aids + tint).
   */
  void setControlHighlight(
      const std::string& control_name,
      InteractiveMarkerControl::ControlHighlight highlight,
      InteractiveMarkerControl::HighlightKind kind =
          InteractiveMarkerControl::HighlightKind::kAuto);

  bool hasMenu() const { return has_menu_; }
  std::shared_ptr<QMenu> getMenu() const { return menu_; }

  /**
   * @brief Builds feedback for the current pose and emits @ref userFeedback.
   */
  void publishFeedback(
      automsgs::msgs::visualization_msgs::InteractiveMarkerFeedback& feedback,
      bool mouse_point_valid = false,
      const QVector3D& mouse_point_rel_world = QVector3D());

  /**
   * @brief Shows the context menu at @p global_pos (Qt screen coords).
   */
  void showMenu(const QPoint& global_pos, const std::string& control_name,
                const QVector3D& three_d_point, bool valid_point);

  /**
   * @brief Draws this marker and its controls into @p scene.
   */
  void draw(rendering::SceneOverlay& scene,
            const common::DisplayPropertyMap& properties) const;

  /**
   * @brief Snapshot of the current message with live pose applied.
   */
  automsgs::msgs::visualization_msgs::InteractiveMarker toMessage() const;

  const std::map<std::string, InteractiveMarkerControl::SharedPtr>& controls()
      const {
    return controls_;
  }

 signals:
  void userFeedback(
      automsgs::msgs::visualization_msgs::InteractiveMarkerFeedback& feedback);
  void statusUpdate(int level, const std::string& name,
                    const std::string& text);

 private slots:
  void handleMenuSelect(int menu_item_id);

 private:
  void publishPose();
  void updateControls(
      const google::protobuf::RepeatedPtrField<
          automsgs::msgs::visualization_msgs::InteractiveMarkerControl>&
          controls);
  void createMenu(
      const google::protobuf::RepeatedPtrField<
          automsgs::msgs::visualization_msgs::MenuEntry>& entries);
  void populateMenu(QMenu* menu, std::vector<uint32_t>& ids);
  QString makeMenuString(const std::string& entry) const;
  void updateReferencePose();

  common::DisplayContext* context_ = nullptr;

  std::string reference_frame_;
  bool frame_locked_ = true;

  QVector3D position_;
  QQuaternion orientation_ = {1.f, 0.f, 0.f, 0.f};
  bool pose_changed_ = false;

  std::map<std::string, InteractiveMarkerControl::SharedPtr> controls_;
  InteractiveMarkerControl::SharedPtr description_control_;

  std::string name_;
  std::string description_;
  float scale_ = 1.f;

  bool dragging_ = false;
  bool pose_update_requested_ = false;
  QVector3D requested_position_;
  QQuaternion requested_orientation_;

  std::shared_ptr<QMenu> menu_;
  bool has_menu_ = false;
  bool show_description_ = true;
  bool show_axes_ = false;
  bool show_visual_aids_ = false;

  struct MenuNode {
    automsgs::msgs::visualization_msgs::MenuEntry entry;
    std::vector<uint32_t> child_ids;
  };
  std::map<uint32_t, MenuNode> menu_entries_;
  std::vector<uint32_t> top_level_menu_ids_;
  std::string last_control_name_;

  bool got_3d_point_for_menu_ = false;
  QVector3D three_d_point_for_menu_;

  /** Full last message (controls / menu); pose fields kept in sync. */
  automsgs::msgs::visualization_msgs::InteractiveMarker message_;

  mutable std::recursive_mutex mutex_;
};

}  // namespace display
}  // namespace autoviz
