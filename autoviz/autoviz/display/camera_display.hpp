/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <QImage>

#include "autolink/message/raw_message.hpp"
#include "autolink/node/reader.hpp"
#include <automsgs/msgs/sensor_msgs/camera_info.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/display.hpp"
#include "autoviz/integration/message_queue.hpp"

namespace autoviz {
namespace display {

class CameraDisplay : public Display {
 public:
  explicit CameraDisplay(std::string image_channel);

  std::string typeId() const override { return "Camera"; }
  std::string channel() const override { return image_channel_; }
  void setChannel(const std::string& channel) override;

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;
  const QImage& image() const { return image_; }

 protected:
  void onEnable() override;
  void onDisable() override;
  void reset() override;
  void onUpdate() override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  void processImage(
      const automsgs::msgs::sensor_msgs::Image& message);
  void processCameraInfo(
      const automsgs::msgs::sensor_msgs::CameraInfo& message);

  std::string image_channel_;
  std::string camera_info_channel_;
  integration::MessageQueue image_queue_;
  integration::MessageQueue camera_info_queue_;
  std::shared_ptr<autolink::Reader<autolink::message::RawMessage>> image_reader_;
  std::shared_ptr<autolink::Reader<autolink::message::RawMessage>>
      camera_info_reader_;
  QImage image_;
  automsgs::msgs::sensor_msgs::CameraInfo camera_info_;
  bool have_camera_info_ = false;
};

}  // namespace display
}  // namespace autoviz
