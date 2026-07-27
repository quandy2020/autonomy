/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <vector>

#include <QColor>
#include <QImage>
#include <QVector3D>

#include <automsgs/msgs/sensor_msgs/camera_info.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>

namespace autoviz {
namespace display {

struct DepthCloudPoint {
  QVector3D position;
  QColor color;
};

/** Project a depth image into optical-frame points (ROS: x right, y down, z forward). */
std::vector<DepthCloudPoint> projectDepthImage(
    const automsgs::msgs::sensor_msgs::Image& depth,
    const automsgs::msgs::sensor_msgs::CameraInfo& info,
    const QImage* color_image, uint32_t decimation, const QColor& flat_color);

}  // namespace display
}  // namespace autoviz
