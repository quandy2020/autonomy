/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/plugins/displays/depth_cloud/depth_cloud_display.hpp"

#include "autoviz/commsgs/time_utils.hpp"
#include "autoviz/common/display_property.hpp"
#include "autoviz/common/interaction/channel_payload.hpp"
#include "autoviz/plugins/displays/camera/camera_utils.hpp"
#include "autoviz/plugins/displays/depth_cloud/depth_cloud_utils.hpp"
#include "autoviz/plugins/displays/common/ogre_colored_points_draw.hpp"
#include "autoviz/plugins/displays/image/image_utils.hpp"
#include "autoviz/plugins/displays/common/transform_utils.hpp"
#include "autoviz/rendering/point_cloud_style_utils.hpp"

namespace autoviz {
namespace display {

DepthCloudDisplay::DepthCloudDisplay(std::string depth_channel)
    : depth_channel_(std::move(depth_channel)) {}

void DepthCloudDisplay::setChannel(const std::string& channel) {
  if (depth_channel_ == channel) {
    return;
  }
  const bool active = enabled();
  if (active) {
    onDisable();
  }
  depth_channel_ = channel;
  if (active) {
    onEnable();
  }
}

std::vector<common::DisplayPropertySpec> DepthCloudDisplay::propertySpecs() const {
  return {{"camera_info_channel", "Camera Info Topic", "", {},
           common::DisplayPropertyKind::kChannel},
          {"color_channel", "Color Image Topic", "", {},
           common::DisplayPropertyKind::kChannel},
          {"color", "Color", "200;200;200", {}, common::DisplayPropertyKind::kColor},
          {"decimate", "Decimate", "2", {}},
          {"point_size", "Point Size", "2", {}},
          {"style", "Style", "Squares",
           {"Points", "Squares", "Flat Squares", "Spheres", "Tiles", "Boxes"}}};
}

void DepthCloudDisplay::onEnable() {
  if (context_ == nullptr || context_->autolink == nullptr ||
      context_->autolink->node() == nullptr) {
    return;
  }
  camera_info_channel_ =
      propertyValue("camera_info_channel", "");
  color_channel_ = propertyValue("color_channel", "");
  auto& registry = integration::ChannelReaderRegistry::instance();
  if (!depth_channel_.empty() && depth_subscription_id_ == 0) {
    depth_subscription_id_ = registry.subscribe(
        depth_channel_,
        [this](const std::string& payload) { depth_queue_.push(payload); });
  }
  if (!camera_info_channel_.empty() && camera_info_subscription_id_ == 0) {
    camera_info_subscription_id_ = registry.subscribe(
        camera_info_channel_, [this](const std::string& payload) {
          camera_info_queue_.push(payload);
        });
  }
  if (!color_channel_.empty() && color_subscription_id_ == 0) {
    color_subscription_id_ = registry.subscribe(
        color_channel_,
        [this](const std::string& payload) { color_queue_.push(payload); });
  }
}

void DepthCloudDisplay::onDisable() {
  auto& registry = integration::ChannelReaderRegistry::instance();
  if (depth_subscription_id_ != 0) {
    registry.unsubscribe(depth_subscription_id_);
    depth_subscription_id_ = 0;
  }
  if (camera_info_subscription_id_ != 0) {
    registry.unsubscribe(camera_info_subscription_id_);
    camera_info_subscription_id_ = 0;
  }
  if (color_subscription_id_ != 0) {
    registry.unsubscribe(color_subscription_id_);
    color_subscription_id_ = 0;
  }
}

void DepthCloudDisplay::reset() {
  Display::reset();
  depth_queue_.clear();
  camera_info_queue_.clear();
  color_queue_.clear();
  have_depth_ = false;
  have_camera_info_ = false;
  color_image_ = QImage();
  points_.clear();
  if (context_ != nullptr && context_->request_redraw) {
    context_->request_redraw();
  }
}

void DepthCloudDisplay::onUpdate() {
  while (auto payload = camera_info_queue_.pop()) {
    automsgs::msgs::sensor_msgs::CameraInfo info;
    if (integration::ParseChannelMessage(*payload, &info)) {
      camera_info_ = info;
      have_camera_info_ = true;
    }
  }
  while (auto payload = color_queue_.pop()) {
    automsgs::msgs::sensor_msgs::Image image;
    if (integration::ParseChannelMessage(*payload, &image)) {
      color_image_ = imageFromProto(image);
    }
  }
  while (auto payload = depth_queue_.pop()) {
    automsgs::msgs::sensor_msgs::Image image;
    if (integration::ParseChannelMessage(*payload, &image)) {
      depth_image_ = image;
      have_depth_ = true;
    }
  }
  if (have_depth_ && have_camera_info_) {
    rebuildPoints();
  }
}

void DepthCloudDisplay::rebuildPoints() {
  points_.clear();
  if (context_ == nullptr) {
    return;
  }

  const uint32_t decimate = static_cast<uint32_t>(std::max(
      1.f, common::ParseFloatProperty(propertyValue("decimate", "2"), 2.f)));
  const QColor flat_color =
      common::ParseColorProperty(propertyValue("color", "200;200;200"));
  const QImage* color_ptr = color_image_.isNull() ? nullptr : &color_image_;
  const auto optical_points =
      projectDepthImage(depth_image_, camera_info_, color_ptr, decimate, flat_color);
  if (optical_points.empty()) {
    return;
  }

  const auto zero_time = autoviz::commsgs::ZeroTime();
  const std::string frame = camera_info_.header().frame_id().empty()
                                ? depth_image_.header().frame_id()
                                : camera_info_.header().frame_id();
  if (frame.empty()) {
    return;
  }

  try {
    const auto tf = context_->tf_buffer->lookupTransform(context_->fixed_frame,
                                                          frame, zero_time);
    const QMatrix4x4 transform = opticalToWorld(camera_info_, transformToMatrix(tf));
    points_.reserve(optical_points.size());
    for (const auto& point : optical_points) {
      points_.push_back({transform.map(point.position), point.color});
    }
  } catch (...) {
    points_.reserve(optical_points.size());
    for (const auto& point : optical_points) {
      points_.push_back({point.position, point.color});
    }
  }

  if (context_->request_redraw) {
    context_->request_redraw();
  }
}

void DepthCloudDisplay::onDraw(rendering::SceneOverlay& scene) {
  if (points_.empty()) {
    return;
  }
  const float point_size =
      common::ParseFloatProperty(propertyValue("point_size", "2"), 2.f);
  const rendering::PointCloudStyle style = rendering::parsePointCloudStyle(
      propertyValue("style", "Squares"));
  std::vector<ColoredPoint3D> colored;
  colored.reserve(points_.size());
  for (const auto& point : points_) {
    colored.push_back({point.position, point.color});
  }
  drawColoredPointsOgreOrGl(context_, scene, name(), typeId(), point_size, style,
                          colored, true);
}

}  // namespace display
}  // namespace autoviz
