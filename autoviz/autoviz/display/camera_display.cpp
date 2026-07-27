/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#include "autoviz/display/camera_display.hpp"

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include "autoviz/commsgs/time_utils.hpp"
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/camera_utils.hpp"
#include "autoviz/display/image_utils.hpp"
#include "autoviz/display/transform_utils.hpp"

namespace autoviz {
namespace display {

CameraDisplay::CameraDisplay(std::string image_channel)
    : image_channel_(std::move(image_channel)) {}

void CameraDisplay::setChannel(const std::string& channel) {
  if (image_channel_ == channel) {
    return;
  }
  const bool active = enabled();
  if (active) {
    onDisable();
  }
  image_channel_ = channel;
  if (active) {
    onEnable();
  }
}

std::vector<common::DisplayPropertySpec> CameraDisplay::propertySpecs() const {
  return {{"camera_info_channel", "Camera Info Topic", "/fake/camera_info", {},
           common::DisplayPropertyKind::kChannel},
          {"near_distance", "Near Clip", "0.2", {}},
          {"far_distance", "Far Clip", "3.0", {}},
          {"color", "Color", "255;180;80", {}, common::DisplayPropertyKind::kColor}};
}

void CameraDisplay::onEnable() {
  if (context_ == nullptr || context_->autolink == nullptr ||
      context_->autolink->node() == nullptr) {
    return;
  }
  camera_info_channel_ =
      propertyValue("camera_info_channel", "/fake/camera_info");
  image_reader_ =
      context_->autolink->node()->CreateReader<autolink::message::RawMessage>(
          image_channel_,
          [this](const std::shared_ptr<autolink::message::RawMessage>& msg) {
            if (msg != nullptr) {
              image_queue_.push(msg->message);
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
}

void CameraDisplay::onDisable() {
  image_reader_.reset();
  camera_info_reader_.reset();
}

void CameraDisplay::onUpdate() {
  while (auto payload = camera_info_queue_.pop()) {
    automsgs::msgs::sensor_msgs::CameraInfo info;
    if (info.ParseFromString(*payload)) {
      processCameraInfo(info);
    }
  }
  while (auto payload = image_queue_.pop()) {
    automsgs::msgs::sensor_msgs::Image image;
    if (image.ParseFromString(*payload)) {
      processImage(image);
    }
  }
}

void CameraDisplay::processImage(
    const automsgs::msgs::sensor_msgs::Image& message) {
  image_ = imageFromProto(message);
  if (context_ != nullptr && context_->image_updated && !image_.isNull()) {
    context_->image_updated(name(), image_);
  }
}

void CameraDisplay::processCameraInfo(
    const automsgs::msgs::sensor_msgs::CameraInfo& message) {
  camera_info_ = message;
  have_camera_info_ = true;
  if (context_ != nullptr && context_->request_redraw) {
    context_->request_redraw();
  }
}

void CameraDisplay::onDraw(rendering::SceneOverlay& scene) {
  if (!have_camera_info_ || context_ == nullptr || context_->tf_buffer == nullptr) {
    return;
  }

  const auto zero_time = autoviz::commsgs::ZeroTime();
  const std::string frame = camera_info_.header().frame_id().empty()
                                ? context_->fixed_frame
                                : camera_info_.header().frame_id();
  automsgs::msgs::geometry_msgs::TransformStamped tf;
  try {
    tf = context_->tf_buffer->lookupTransform(context_->fixed_frame, frame,
                                              zero_time);
  } catch (...) {
    return;
  }

  QMatrix4x4 camera_to_fixed = transformToMatrix(tf);

  const float near_distance =
      common::ParseFloatProperty(propertyValue("near_distance", "0.2"), 0.2f);
  const float far_distance =
      common::ParseFloatProperty(propertyValue("far_distance", "3.0"), 3.0f);
  const QColor color =
      common::ParseColorProperty(propertyValue("color", "255;180;80"),
                                 QColor(255, 180, 80));
  const QMatrix4x4 transform =
      opticalToWorld(camera_info_, camera_to_fixed);
  for (const auto& segment :
       buildCameraFrustumSegments(camera_info_, near_distance, far_distance)) {
    scene.addLine(transform.map(segment.first), transform.map(segment.second),
                  color);
  }

  if (!image_.isNull()) {
    const CameraFarPlane plane =
        buildCameraFarPlane(camera_info_, far_distance);
    if (plane.valid) {
      scene.addTexturedQuad(transform.map(plane.top_left),
                            transform.map(plane.top_right),
                            transform.map(plane.bottom_right),
                            transform.map(plane.bottom_left), image_);
    }
  }
}

}  // namespace display
}  // namespace autoviz
