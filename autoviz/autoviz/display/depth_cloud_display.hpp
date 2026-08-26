/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <QColor>
#include <QImage>
#include <QVector3D>

#include "autolink/message/raw_message.hpp"
#include "autolink/node/reader.hpp"
#include <automsgs/msgs/sensor_msgs/camera_info.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/display.hpp"
#include "autoviz/integration/message_queue.hpp"

namespace autoviz {
namespace display {

class DepthCloudDisplay : public Display {
 public:
  explicit DepthCloudDisplay(std::string depth_channel);

  std::string typeId() const override { return "DepthCloud"; }
  std::string channel() const override { return depth_channel_; }
  void setChannel(const std::string& channel) override;

  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  void onEnable() override;
  void onDisable() override;
  void reset() override;
  void onUpdate() override;
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  struct ColoredPoint {
    QVector3D position;
    QColor color;
  };

  void rebuildPoints();

  std::string depth_channel_;
  std::string camera_info_channel_;
  std::string color_channel_;
  integration::MessageQueue depth_queue_;
  integration::MessageQueue camera_info_queue_;
  integration::MessageQueue color_queue_;
  std::shared_ptr<autolink::Reader<autolink::message::RawMessage>> depth_reader_;
  std::shared_ptr<autolink::Reader<autolink::message::RawMessage>>
      camera_info_reader_;
  std::shared_ptr<autolink::Reader<autolink::message::RawMessage>> color_reader_;
  automsgs::msgs::sensor_msgs::Image depth_image_;
  automsgs::msgs::sensor_msgs::CameraInfo camera_info_;
  QImage color_image_;
  bool have_depth_ = false;
  bool have_camera_info_ = false;
  std::vector<ColoredPoint> points_;
};

}  // namespace display
}  // namespace autoviz
