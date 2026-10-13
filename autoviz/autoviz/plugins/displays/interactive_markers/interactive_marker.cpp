/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_default_plugins InteractiveMarker
 * (Willow Garage / Open Source Robotics Foundation).
 *****************************************************************************/

#include "autoviz/plugins/displays/interactive_markers/interactive_marker.hpp"

#include <sstream>

#include <QPoint>

#include <automsgs/msgs/visualization_msgs/menu_entry.pb.h>
#include "autoviz/common/display_context.hpp"
#include "autoviz/plugins/displays/interactive_markers/integer_action.hpp"
#include "autoviz/plugins/displays/common/arrow_mesh_utils.hpp"
#include "autoviz/plugins/displays/common/ogre_mesh_draw.hpp"
#include "autoviz/plugins/displays/marker/marker_draw_utils.hpp"
#include "autoviz/rendering/scene_overlay.hpp"

namespace autoviz {
namespace display {

InteractiveMarker::InteractiveMarker(common::DisplayContext* context)
    : context_(context) {}

bool InteractiveMarker::processMessage(
    const automsgs::msgs::visualization_msgs::InteractiveMarker& message) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  message_ = message;
  name_ = message.name();
  description_ = message.description();

  if (message.controls_size() == 0) {
    emit statusUpdate(0, name_, "Marker empty.");
    return false;
  }

  scale_ = static_cast<float>(message.scale());
  reference_frame_ = message.header().frame_id();
  frame_locked_ = (message.header().stamp().sec() == 0 &&
                   message.header().stamp().nanosec() == 0);

  position_ = QVector3D(static_cast<float>(message.pose().position().x()),
                        static_cast<float>(message.pose().position().y()),
                        static_cast<float>(message.pose().position().z()));
  orientation_ =
      QQuaternion(static_cast<float>(message.pose().orientation().w()),
                  static_cast<float>(message.pose().orientation().x()),
                  static_cast<float>(message.pose().orientation().y()),
                  static_cast<float>(message.pose().orientation().z()));
  if (orientation_.isNull()) {
    orientation_ = QQuaternion(1.f, 0.f, 0.f, 0.f);
  }
  pose_changed_ = false;
  has_menu_ = message.menu_entries_size() > 0;

  // Bake ROS autoComplete handles into the message so registry picking and
  // draw share the same ARROW / TRIANGLE_LIST geometry (RViz parity).
  autoCompleteInteractiveMarker(&message_);

  updateReferencePose();
  updateControls(message_.controls());

  // Title / description as a VIEW_FACING TEXT control (RViz makeTitle).
  description_control_ =
      std::make_shared<InteractiveMarkerControl>(context_, this);
  automsgs::msgs::visualization_msgs::InteractiveMarkerControl title;
  title.set_name("___description___");
  title.set_always_visible(true);
  title.set_orientation_mode(
      automsgs::msgs::visualization_msgs::InteractiveMarkerControl::VIEW_FACING);
  title.set_interaction_mode(
      automsgs::msgs::visualization_msgs::InteractiveMarkerControl::NONE);
  automsgs::msgs::visualization_msgs::Marker text;
  text.set_type(automsgs::msgs::visualization_msgs::Marker::TEXT_VIEW_FACING);
  text.set_text(description_.empty() ? name_ : description_);
  text.mutable_scale()->set_z(std::max(0.08, static_cast<double>(scale_) * 0.15));
  text.mutable_color()->set_r(1.f);
  text.mutable_color()->set_g(1.f);
  text.mutable_color()->set_b(1.f);
  text.mutable_color()->set_a(1.f);
  // RViz interactive_markers::makeTitle: z = 1.4 * scale.
  text.mutable_pose()->mutable_position()->set_z(scale_ * 1.4);
  *title.add_markers() = text;
  description_control_->processMessage(title);
  description_control_->setVisible(show_description_);

  menu_entries_.clear();
  menu_.reset();
  if (has_menu_) {
    createMenu(message.menu_entries());
  }

  if (frame_locked_) {
    std::ostringstream s;
    s << "Locked to frame " << reference_frame_;
    emit statusUpdate(0, name_, s.str());
  } else {
    emit statusUpdate(0, name_, "Position is fixed.");
  }
  return true;
}

void InteractiveMarker::processMessage(
    const automsgs::msgs::visualization_msgs::InteractiveMarkerPose& message) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  QVector3D position(static_cast<float>(message.pose().position().x()),
                     static_cast<float>(message.pose().position().y()),
                     static_cast<float>(message.pose().position().z()));
  QQuaternion orientation(
      static_cast<float>(message.pose().orientation().w()),
      static_cast<float>(message.pose().orientation().x()),
      static_cast<float>(message.pose().orientation().y()),
      static_cast<float>(message.pose().orientation().z()));
  if (orientation.isNull()) {
    orientation = QQuaternion(1.f, 0.f, 0.f, 0.f);
  }
  if (!message.header().frame_id().empty()) {
    reference_frame_ = message.header().frame_id();
  }
  frame_locked_ = (message.header().stamp().sec() == 0 &&
                   message.header().stamp().nanosec() == 0);
  requestPoseUpdate(position, orientation);
}

void InteractiveMarker::update() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (frame_locked_) {
    updateReferencePose();
  }
  for (auto& [name, control] : controls_) {
    (void)name;
    control->update();
  }
  if (description_control_) {
    description_control_->update();
  }
  if (dragging_ && pose_changed_) {
    publishPose();
  }
}

void InteractiveMarker::setPose(const QVector3D& position,
                                const QQuaternion& orientation,
                                const std::string& control_name) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  position_ = position;
  orientation_ = orientation;
  pose_changed_ = true;
  last_control_name_ = control_name;

  message_.mutable_pose()->mutable_position()->set_x(position_.x());
  message_.mutable_pose()->mutable_position()->set_y(position_.y());
  message_.mutable_pose()->mutable_position()->set_z(position_.z());
  message_.mutable_pose()->mutable_orientation()->set_w(orientation_.scalar());
  message_.mutable_pose()->mutable_orientation()->set_x(orientation_.x());
  message_.mutable_pose()->mutable_orientation()->set_y(orientation_.y());
  message_.mutable_pose()->mutable_orientation()->set_z(orientation_.z());

  for (auto& [name, control] : controls_) {
    (void)name;
    control->interactiveMarkerPoseChanged(position_, orientation_);
  }
  if (description_control_) {
    description_control_->interactiveMarkerPoseChanged(position_, orientation_);
  }
}

void InteractiveMarker::translate(const QVector3D& delta_position,
                                  const std::string& control_name) {
  setPose(position_ + delta_position, orientation_, control_name);
}

void InteractiveMarker::rotate(const QQuaternion& delta_orientation,
                               const std::string& control_name) {
  setPose(position_, delta_orientation * orientation_, control_name);
}

void InteractiveMarker::requestPoseUpdate(const QVector3D& position,
                                          const QQuaternion& orientation) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (dragging_) {
    pose_update_requested_ = true;
    requested_position_ = position;
    requested_orientation_ = orientation;
  } else {
    updateReferencePose();
    setPose(position, orientation, "");
  }
}

void InteractiveMarker::startDragging() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  dragging_ = true;
  pose_changed_ = false;
}

void InteractiveMarker::stopDragging() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  dragging_ = false;
  if (pose_update_requested_) {
    updateReferencePose();
    setPose(requested_position_, requested_orientation_, "");
    pose_update_requested_ = false;
  }
}

void InteractiveMarker::setShowDescription(bool show) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  show_description_ = show;
  if (description_control_) {
    description_control_->setVisible(show);
  }
}

void InteractiveMarker::setShowAxes(bool show) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  show_axes_ = show;
}

void InteractiveMarker::setShowVisualAids(bool show) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  show_visual_aids_ = show;
  for (auto& [name, control] : controls_) {
    (void)name;
    control->setShowVisualAids(show);
  }
}

void InteractiveMarker::clearControlHighlights() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  for (auto& [name, control] : controls_) {
    (void)name;
    control->setHighlight(InteractiveMarkerControl::kNoHighlight);
  }
}

void InteractiveMarker::setControlHighlight(
    const std::string& control_name,
    InteractiveMarkerControl::ControlHighlight highlight,
    InteractiveMarkerControl::HighlightKind kind) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  auto it = controls_.find(control_name);
  if (it != controls_.end()) {
    it->second->setHighlight(highlight, kind);
  }
}

void InteractiveMarker::publishFeedback(
    automsgs::msgs::visualization_msgs::InteractiveMarkerFeedback& feedback,
    bool mouse_point_valid, const QVector3D& mouse_point_rel_world) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  feedback.set_marker_name(name_);
  feedback.set_control_name(last_control_name_);

  if (frame_locked_) {
    feedback.mutable_header()->set_frame_id(reference_frame_);
    feedback.mutable_pose()->mutable_position()->set_x(position_.x());
    feedback.mutable_pose()->mutable_position()->set_y(position_.y());
    feedback.mutable_pose()->mutable_position()->set_z(position_.z());
    feedback.mutable_pose()->mutable_orientation()->set_w(orientation_.scalar());
    feedback.mutable_pose()->mutable_orientation()->set_x(orientation_.x());
    feedback.mutable_pose()->mutable_orientation()->set_y(orientation_.y());
    feedback.mutable_pose()->mutable_orientation()->set_z(orientation_.z());
  } else {
    feedback.mutable_header()->set_frame_id(
        context_ != nullptr ? context_->fixed_frame : reference_frame_);
    feedback.mutable_pose()->mutable_position()->set_x(position_.x());
    feedback.mutable_pose()->mutable_position()->set_y(position_.y());
    feedback.mutable_pose()->mutable_position()->set_z(position_.z());
    feedback.mutable_pose()->mutable_orientation()->set_w(orientation_.scalar());
    feedback.mutable_pose()->mutable_orientation()->set_x(orientation_.x());
    feedback.mutable_pose()->mutable_orientation()->set_y(orientation_.y());
    feedback.mutable_pose()->mutable_orientation()->set_z(orientation_.z());
  }

  feedback.set_mouse_point_valid(mouse_point_valid);
  if (mouse_point_valid) {
    feedback.mutable_mouse_point()->set_x(mouse_point_rel_world.x());
    feedback.mutable_mouse_point()->set_y(mouse_point_rel_world.y());
    feedback.mutable_mouse_point()->set_z(mouse_point_rel_world.z());
  }
  emit userFeedback(feedback);
}

void InteractiveMarker::showMenu(const QPoint& global_pos,
                                 const std::string& control_name,
                                 const QVector3D& three_d_point,
                                 bool valid_point) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!menu_) {
    return;
  }
  got_3d_point_for_menu_ = valid_point;
  three_d_point_for_menu_ = three_d_point;
  last_control_name_ = control_name;
  menu_->exec(global_pos);
}

void InteractiveMarker::publishPose() {
  automsgs::msgs::visualization_msgs::InteractiveMarkerFeedback feedback;
  feedback.set_event_type(
      automsgs::msgs::visualization_msgs::InteractiveMarkerFeedback::
          POSE_UPDATE);
  publishFeedback(feedback);
  pose_changed_ = false;
}

void InteractiveMarker::updateControls(
    const google::protobuf::RepeatedPtrField<
        automsgs::msgs::visualization_msgs::InteractiveMarkerControl>&
        controls) {
  controls_.clear();
  for (const auto& control_msg : controls) {
    auto control =
        std::make_shared<InteractiveMarkerControl>(context_, this);
    control->processMessage(control_msg);
    control->setShowVisualAids(show_visual_aids_);
    std::string key = control_msg.name();
    if (key.empty()) {
      key = "control_" + std::to_string(controls_.size());
    }
    controls_[key] = std::move(control);
  }
}

void InteractiveMarker::createMenu(
    const google::protobuf::RepeatedPtrField<
        automsgs::msgs::visualization_msgs::MenuEntry>& entries) {
  menu_entries_.clear();
  top_level_menu_ids_.clear();
  for (const auto& entry : entries) {
    MenuNode node;
    node.entry = entry;
    menu_entries_[entry.id()] = node;
  }
  for (auto& [id, node] : menu_entries_) {
    (void)id;
    if (node.entry.parent_id() == 0) {
      top_level_menu_ids_.push_back(node.entry.id());
    } else {
      auto parent = menu_entries_.find(node.entry.parent_id());
      if (parent != menu_entries_.end()) {
        parent->second.child_ids.push_back(node.entry.id());
      }
    }
  }
  menu_ = std::make_shared<QMenu>();
  populateMenu(menu_.get(), top_level_menu_ids_);
}

void InteractiveMarker::populateMenu(QMenu* menu, std::vector<uint32_t>& ids) {
  if (menu == nullptr) {
    return;
  }
  for (uint32_t id : ids) {
    auto it = menu_entries_.find(id);
    if (it == menu_entries_.end()) {
      continue;
    }
    MenuNode& node = it->second;
    const QString title = makeMenuString(node.entry.title());
    if (!node.child_ids.empty()) {
      QMenu* submenu = menu->addMenu(title);
      populateMenu(submenu, node.child_ids);
    } else {
      auto* action =
          new IntegerAction(title, menu, static_cast<int>(node.entry.id()));
      menu->addAction(action);
      connect(action, &IntegerAction::triggered, this,
              &InteractiveMarker::handleMenuSelect);
    }
  }
}

QString InteractiveMarker::makeMenuString(const std::string& entry) const {
  QString text = QString::fromStdString(entry);
  text.replace(QLatin1String("[+]"), QString());
  return text.trimmed();
}

void InteractiveMarker::handleMenuSelect(int menu_item_id) {
  automsgs::msgs::visualization_msgs::InteractiveMarkerFeedback feedback;
  feedback.set_event_type(
      automsgs::msgs::visualization_msgs::InteractiveMarkerFeedback::
          MENU_SELECT);
  feedback.set_menu_entry_id(static_cast<uint32_t>(menu_item_id));
  publishFeedback(feedback, got_3d_point_for_menu_, three_d_point_for_menu_);
}

void InteractiveMarker::updateReferencePose() {
  // Pose is already stored in the marker's reference frame; TF is applied at
  // draw / control-transform time via DisplayContext.
}

void InteractiveMarker::draw(rendering::SceneOverlay& scene,
                             const common::DisplayPropertyMap& properties) const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  const float scale = std::max(scale_, 0.05f);

  if (show_axes_) {
    // RViz rviz_rendering::Axes(length=scale, radius=scale*0.05).
    automsgs::msgs::visualization_msgs::Marker pose_marker;
    pose_marker.mutable_header()->set_frame_id(reference_frame_);
    pose_marker.mutable_pose()->mutable_position()->set_x(position_.x());
    pose_marker.mutable_pose()->mutable_position()->set_y(position_.y());
    pose_marker.mutable_pose()->mutable_position()->set_z(position_.z());
    pose_marker.mutable_pose()->mutable_orientation()->set_w(
        orientation_.scalar());
    pose_marker.mutable_pose()->mutable_orientation()->set_x(orientation_.x());
    pose_marker.mutable_pose()->mutable_orientation()->set_y(orientation_.y());
    pose_marker.mutable_pose()->mutable_orientation()->set_z(orientation_.z());
    const QMatrix4x4 axes_tf =
        markerPoseTransformInFixedFrame(pose_marker, context_);
    const QVector3D origin = axes_tf.map(QVector3D(0.f, 0.f, 0.f));
    const QVector3D x_axis =
        axes_tf.map(QVector3D(scale, 0.f, 0.f)) - origin;
    const QVector3D y_axis =
        axes_tf.map(QVector3D(0.f, scale, 0.f)) - origin;
    const QVector3D z_axis =
        axes_tf.map(QVector3D(0.f, 0.f, scale)) - origin;
    const float diameter = scale * 0.1f;
    std::vector<ColoredMeshInstance> axes_meshes;
    appendSolidArrowMeshes(&axes_meshes, origin, origin + x_axis,
                           QColor(255, 0, 0), 0.23f, diameter, diameter * 1.6f);
    appendSolidArrowMeshes(&axes_meshes, origin, origin + y_axis,
                           QColor(0, 255, 0), 0.23f, diameter, diameter * 1.6f);
    appendSolidArrowMeshes(&axes_meshes, origin, origin + z_axis,
                           QColor(0, 0, 255), 0.23f, diameter, diameter * 1.6f);
    drawMeshesOgreOrGl(context_, scene, name_ + "/axes", axes_meshes);
  }

  int index = 0;
  for (const auto& [control_name, control] : controls_) {
    control->draw(scene, properties,
                  name_ + "/control/" + control_name + "/" +
                      std::to_string(index++));
  }
  if (description_control_ && show_description_) {
    description_control_->draw(scene, properties, name_ + "/description");
  }
}

automsgs::msgs::visualization_msgs::InteractiveMarker
InteractiveMarker::toMessage() const {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  automsgs::msgs::visualization_msgs::InteractiveMarker msg = message_;
  msg.set_name(name_);
  msg.mutable_header()->set_frame_id(reference_frame_);
  msg.mutable_pose()->mutable_position()->set_x(position_.x());
  msg.mutable_pose()->mutable_position()->set_y(position_.y());
  msg.mutable_pose()->mutable_position()->set_z(position_.z());
  msg.mutable_pose()->mutable_orientation()->set_w(orientation_.scalar());
  msg.mutable_pose()->mutable_orientation()->set_x(orientation_.x());
  msg.mutable_pose()->mutable_orientation()->set_y(orientation_.y());
  msg.mutable_pose()->mutable_orientation()->set_z(orientation_.z());
  msg.set_scale(scale_);
  return msg;
}

}  // namespace display
}  // namespace autoviz
