/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/depth_cloud_display.hpp"

#include "autoviz/commsgs/time_utils.hpp"
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/camera_utils.hpp"
#include "autoviz/display/depth_cloud_utils.hpp"
#include "autoviz/display/ogre_colored_points_draw.hpp"
#include "autoviz/display/image_utils.hpp"
#include "autoviz/display/transform_utils.hpp"
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
  return {{"camera_info_channel", "Camera Info Topic", "/fake/camera_info", {},
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
      propertyValue("camera_info_channel", "/fake/camera_info");
  color_channel_ = propertyValue("color_channel", "");
  depth_reader_ =
      context_->autolink->node()->CreateReader<autolink::message::RawMessage>(
          depth_channel_,
          [this](const std::shared_ptr<autolink::message::RawMessage>& msg) {
            if (msg != nullptr) {
              depth_queue_.push(msg->message);
            }
          });
  camera_info_reader_ =
      context_->autolink->node()->CreateReader<autolink::message::RawMessage>(
          camera_info_channel_,
          [this](const std::shared_ptr<autolink::message::RawMessage>& msg) {
            if (msg != nullptr) {
              camera_info_queue_.push(msg->message);
            }
          });
  if (!color_channel_.empty()) {
    color_reader_ =
        context_->autolink->node()->CreateReader<autolink::message::RawMessage>(
            color_channel_,
            [this](const std::shared_ptr<autolink::message::RawMessage>& msg) {
              if (msg != nullptr) {
                color_queue_.push(msg->message);
              }
            });
  }
}

void DepthCloudDisplay::onDisable() {
  depth_reader_.reset();
  camera_info_reader_.reset();
  color_reader_.reset();
}

void DepthCloudDisplay::onUpdate() {
  while (auto payload = camera_info_queue_.pop()) {
    automsgs::msgs::sensor_msgs::CameraInfo info;
    if (info.ParseFromString(*payload)) {
      camera_info_ = info;
      have_camera_info_ = true;
    }
  }
  while (auto payload = color_queue_.pop()) {
    automsgs::msgs::sensor_msgs::Image image;
    if (image.ParseFromString(*payload)) {
      color_image_ = imageFromProto(image);
    }
  }
  while (auto payload = depth_queue_.pop()) {
    automsgs::msgs::sensor_msgs::Image image;
    if (image.ParseFromString(*payload)) {
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
      points_.push_back({transform.map(point.position()), point.color});
    }
  } catch (...) {
    points_.reserve(optical_points.size());
    for (const auto& point : optical_points) {
      points_.push_back({point.position(), point.color});
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
    colored.push_back({point.position(), point.color});
  }
  drawColoredPointsOgreOrGl(context_, scene, name(), typeId(), point_size, style,
                          colored, true);
}

}  // namespace display
}  // namespace autoviz
