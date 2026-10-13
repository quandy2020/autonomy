/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Adapted from rviz_default_plugins RobotModelDisplay
 * (Willow Garage / Open Source Robotics Foundation).
 *****************************************************************************/

#include "autoviz/plugins/displays/robot_model/robot_model_display.hpp"

#include <cmath>

#include <QFileInfo>

#include "autolink/common/log.hpp"
#include <automsgs/msgs/std_msgs/string.pb.h>
#include "autoviz/plugins/robot/tf_link_updater.hpp"
#include "autoviz/common/interaction/channel_payload.hpp"

namespace autoviz {
namespace display {
namespace {

enum class DescriptionSource { kTopic, kFile };

DescriptionSource ParseDescriptionSource(const std::string& value) {
  return (value == "File") ? DescriptionSource::kFile
                           : DescriptionSource::kTopic;
}

}  // namespace

RobotModelDisplay::RobotModelDisplay(std::string joint_channel)
    : joint_channel_(std::move(joint_channel)),
      robot_(std::make_unique<robot::Robot>()),
      transformer_guard_(
          std::make_unique<transformation::TransformerGuard<
              transformation::TFFrameTransformer>>(this, "TF")) {
  setProperties({});
  description_channel_ =
      propertyValue("description_channel", "/robot_description");
  updateVisualVisible();
  updateCollisionVisible();
  updateAlpha();
  updateMassVisible();
  updateInertiaVisible();
}

void RobotModelDisplay::setChannel(const std::string& channel) {
  if (joint_channel_ == channel) {
    return;
  }
  const bool active = enabled();
  if (active) {
    onDisable();
  }
  joint_channel_ = channel;
  if (active) {
    onEnable();
  }
}

std::vector<common::DisplayPropertySpec> RobotModelDisplay::propertySpecs()
    const {
  // Property names aligned with RViz RobotModelDisplay where possible.
  return {{"description_source", "Description Source", "Topic",
           {"Topic", "File"}},
          {"urdf_path", "Description File", "", {},
           common::DisplayPropertyKind::kPath},
          {"description_channel", "Description Topic", "/robot_description", {},
           common::DisplayPropertyKind::kChannel},
          {"tf_prefix", "TF Prefix", "", {}},
          {"update_interval", "Update Interval", "0"},
          {"root_link", "Root Link", "", {}},
          {"alpha", "Alpha", "1.0"},
          {"visual_enabled", "Visual Enabled", "true", {"true", "false"}},
          {"collision_enabled", "Collision Enabled", "false",
           {"true", "false"}},
          {"show_collision", "Show Collision", "false", {"true", "false"}},
          {"collision_alpha", "Collision Alpha", "0.35"},
          {"mass_enabled", "Mass", "false", {"true", "false"}},
          {"inertia_enabled", "Inertia", "false", {"true", "false"}},
          {"color", "Color", "180;180;180", {},
           common::DisplayPropertyKind::kColor},
          {"visual_style", "Visual Style", "solid", {"solid", "wireframe"}},
          {"show_axes", "Show Axes", "true", {"true", "false"}},
          {"use_urdf_materials", "Use URDF Materials", "true",
           {"true", "false"}}};
}

void RobotModelDisplay::updateVisualVisible() {
  if (robot_ != nullptr) {
    robot_->setVisualVisible(
        common::ParseBoolProperty(propertyValue("visual_enabled", "true"),
                                  true));
  }
}

void RobotModelDisplay::updateCollisionVisible() {
  if (robot_ == nullptr) {
    return;
  }
  const bool enabled =
      common::ParseBoolProperty(propertyValue("collision_enabled", "false"),
                                false) ||
      common::ParseBoolProperty(propertyValue("show_collision", "false"),
                                false);
  robot_->setCollisionVisible(enabled);
}

void RobotModelDisplay::updateAlpha() {
  if (robot_ != nullptr) {
    robot_->setAlpha(
        common::ParseFloatProperty(propertyValue("alpha", "1.0"), 1.0f));
  }
}

void RobotModelDisplay::updateMassVisible() {
  if (robot_ != nullptr) {
    robot_->setMassVisible(
        common::ParseBoolProperty(propertyValue("mass_enabled", "false"),
                                  false));
  }
}

void RobotModelDisplay::updateInertiaVisible() {
  if (robot_ != nullptr) {
    robot_->setInertiaVisible(
        common::ParseBoolProperty(propertyValue("inertia_enabled", "false"),
                                  false));
  }
}

void RobotModelDisplay::onPropertyChanged(const std::string& key) {
  if (key == "visual_enabled") {
    updateVisualVisible();
    return;
  }
  if (key == "collision_enabled" || key == "show_collision") {
    updateCollisionVisible();
    return;
  }
  if (key == "alpha") {
    updateAlpha();
    return;
  }
  if (key == "mass_enabled") {
    updateMassVisible();
    return;
  }
  if (key == "inertia_enabled") {
    updateInertiaVisible();
    return;
  }
  if (key == "urdf_path" || key == "description_source") {
    if (enabled()) {
      load_urdf();
    }
    return;
  }
  if (key != "description_channel") {
    return;
  }
  const std::string next =
      propertyValue("description_channel", "/robot_description");
  if (next == description_channel_) {
    return;
  }
  description_channel_ = next;
  if (enabled()) {
    onDisable();
    onEnable();
  }
}

void RobotModelDisplay::clear() {
  if (robot_ != nullptr) {
    robot_->clear();
  }
  robot_description_.clear();
}

void RobotModelDisplay::load_urdf() {
  const auto source =
      ParseDescriptionSource(propertyValue("description_source", "Topic"));
  if (source == DescriptionSource::kFile) {
    const std::string path = propertyValue("urdf_path", "");
    if (!path.empty()) {
      load_urdf_from_file(path);
    } else {
      clear();
    }
    return;
  }
  // Topic: keep last description; file path in topic payload handled in
  // processDescription. If a file is also set, use it as a warm start.
  const std::string path = propertyValue("urdf_path", "");
  if (!path.empty() && robot_description_.empty()) {
    load_urdf_from_file(path);
  }
}

void RobotModelDisplay::load_urdf_from_file(const std::string& filepath) {
  if (robot_ == nullptr) {
    return;
  }
  if (!robot_->loadFromFile(filepath)) {
    clear();
    setStatusError("URDF is empty or failed to parse");
    return;
  }
  robot_description_ = filepath;
  display_urdf_content();
}

void RobotModelDisplay::load_urdf_from_string(
    const std::string& robot_description) {
  if (robot_description.empty()) {
    clear();
    setStatusError("URDF is empty");
    return;
  }
  if (robot_description == robot_description_) {
    return;
  }
  if (robot_ == nullptr || !robot_->loadFromString(robot_description)) {
    clear();
    setStatusError("URDF failed Model parse");
    return;
  }
  robot_description_ = robot_description;
  display_urdf_content();
}

void RobotModelDisplay::display_urdf_content() {
  if (robot_ == nullptr || robot_->empty()) {
    return;
  }
  setStatusOk("URDF parsed OK");
  if (!robot_->geometryErrors().empty()) {
    AWARN << "robot model geometry errors:" << robot_->geometryErrors();
    setStatusWarn(std::string("Errors loading geometries:") +
                  robot_->geometryErrors());
  } else {
    std::size_t mesh_visuals = 0;
    for (const auto& link : robot_->model().links()) {
      for (const auto& visual : link.visuals) {
        if (visual.type == UrdfGeometry::Type::kMesh) {
          ++mesh_visuals;
        }
      }
      if (link.visuals.empty() && link.has_visual &&
          link.visual.type == UrdfGeometry::Type::kMesh) {
        ++mesh_visuals;
      }
    }
    AINFO << "robot model loaded, links=" << robot_->model().links().size()
          << " mesh_visuals=" << mesh_visuals;
    setStatusOk("Model loaded");
  }
  updateVisualVisible();
  updateCollisionVisible();
  updateAlpha();
  updateRobot();
  if (context_ != nullptr && context_->request_redraw) {
    context_->request_redraw();
  }
}

void RobotModelDisplay::updateRobot() {
  if (robot_ == nullptr || context_ == nullptr) {
    return;
  }
  // RViz RobotModelDisplay::updateRobot — TFLinkUpdater + Robot::update.
  // Per-link status is not mirrored (Autoviz has a single status line); failed
  // lookups clear the link pose so draw falls back to JointState FK.
  const std::string tf_prefix = propertyValue("tf_prefix", "");
  robot::TFLinkUpdater updater(context_, {}, tf_prefix);
  robot_->update(updater);
  has_new_transforms_ = true;
}

void RobotModelDisplay::subscribeChannels() {
  if (context_ == nullptr || context_->autolink == nullptr ||
      context_->autolink->node() == nullptr) {
    return;
  }
  const bool shared_channel =
      !description_channel_.empty() && description_channel_ == joint_channel_;
  auto& registry = integration::ChannelReaderRegistry::instance();
  if (joint_subscription_ == 0 && !joint_channel_.empty()) {
    joint_subscription_ = registry.subscribe(
        joint_channel_,
        [this, shared_channel](const std::string& payload) {
          if (shared_channel) {
            description_queue_.push(payload);
          }
          joint_queue_.push(payload);
        });
  }
  const auto source =
      ParseDescriptionSource(propertyValue("description_source", "Topic"));
  if (source == DescriptionSource::kTopic && description_subscription_ == 0 &&
      !description_channel_.empty() && !shared_channel) {
    description_subscription_ = registry.subscribe(
        description_channel_,
        [this](const std::string& payload) {
          description_queue_.push(payload);
        });
  }
}

void RobotModelDisplay::onEnable() {
  if (context_ == nullptr || context_->autolink == nullptr ||
      context_->autolink->node() == nullptr) {
    setStatusError("Autolink not ready");
    return;
  }
  if (!transformer_guard_initialized_ && transformer_guard_ != nullptr &&
      context_->transformation_manager != nullptr) {
    transformer_guard_->initialize(context_, context_->transformation_manager);
    transformer_guard_initialized_ = true;
  }
  if (transformer_guard_ != nullptr && !transformer_guard_->checkTransformer()) {
    setEnabled(false);
    setStatusError("The display works only with TF Transformer");
    return;
  }
  description_channel_ =
      propertyValue("description_channel", "/robot_description");
  load_urdf();
  if (robot_ != nullptr) {
    robot_->setVisible(true);
  }
  const auto source =
      ParseDescriptionSource(propertyValue("description_source", "Topic"));
  if (source == DescriptionSource::kTopic) {
    subscribeChannels();
  } else {
    // File mode still needs joint states for FK fallback.
    subscribeChannels();
  }
  if (joint_subscription_ == 0 && !joint_channel_.empty()) {
    setStatusError("Failed to subscribe joint states");
  }
}

void RobotModelDisplay::onDisable() {
  auto& registry = integration::ChannelReaderRegistry::instance();
  if (joint_subscription_ != 0) {
    registry.unsubscribe(joint_subscription_);
    joint_subscription_ = 0;
  }
  if (description_subscription_ != 0) {
    registry.unsubscribe(description_subscription_);
    description_subscription_ = 0;
  }
  if (robot_ != nullptr) {
    robot_->setVisible(false);
  }
  clear();
}

void RobotModelDisplay::reset() {
  Display::reset();
  joint_queue_.clear();
  description_queue_.clear();
  has_new_transforms_ = true;
  if (robot_ != nullptr) {
    robot_->setJointPositions({});
  }
  if (context_ != nullptr && context_->request_redraw) {
    context_->request_redraw();
  }
}

void RobotModelDisplay::processDescription(const std::string& urdf_text) {
  load_urdf_from_string(urdf_text);
}

void RobotModelDisplay::processJointState(
    const proto_wire::ParsedJointState& message) {
  if (robot_ == nullptr) {
    return;
  }
  std::unordered_map<std::string, double> positions;
  for (std::size_t i = 0;
       i < message.names.size() && i < message.positions.size(); ++i) {
    positions[message.names[i]] = message.positions[i];
  }
  robot_->setJointPositions(positions);
  has_new_transforms_ = true;
  if (context_ != nullptr && context_->request_redraw) {
    context_->request_redraw();
  }
}

void RobotModelDisplay::onUpdate() {
  const auto source =
      ParseDescriptionSource(propertyValue("description_source", "Topic"));
  if (joint_subscription_ == 0 ||
      (source == DescriptionSource::kTopic && description_subscription_ == 0 &&
       !description_channel_.empty() &&
       description_channel_ != joint_channel_)) {
    subscribeChannels();
  }

  while (auto payload = description_queue_.pop()) {
    const std::string decoded = integration::DecodeChannelPayload(*payload);
    automsgs::msgs::std_msgs::String message;
    if (message.ParseFromString(decoded) || message.ParseFromString(*payload)) {
      processDescription(message.data());
    } else {
      setStatusWarn("Failed to parse robot description");
    }
  }
  while (auto payload = joint_queue_.pop()) {
    const std::string decoded = integration::DecodeChannelPayload(*payload);
    proto_wire::ParsedJointState parsed;
    if (ParseJointStatePayload(decoded, &parsed) ||
        ParseJointStatePayload(*payload, &parsed)) {
      processJointState(parsed);
    }
  }

  // RViz update_rate_property_: 0 means every cycle.
  const double interval_sec =
      common::ParseFloatProperty(propertyValue("update_interval", "0"), 0.f);
  time_since_last_transform_ += std::chrono::milliseconds(16);
  const auto rate = std::chrono::nanoseconds(
      static_cast<std::int64_t>(std::llround(interval_sec * 1e9)));
  const bool due = rate < std::chrono::microseconds(100) ||
                   time_since_last_transform_ >= rate;
  if (has_new_transforms_ || due) {
    updateRobot();
    has_new_transforms_ = false;
    time_since_last_transform_ = std::chrono::nanoseconds(0);
    if (context_ != nullptr && context_->request_redraw) {
      context_->request_redraw();
    }
  }
}

void RobotModelDisplay::onDraw(rendering::SceneOverlay& scene) {
  if (robot_ == nullptr) {
    return;
  }
  updateVisualVisible();
  updateCollisionVisible();
  updateAlpha();
  robot_->draw(scene, context_, properties(), name());
}

}  // namespace display
}  // namespace autoviz
