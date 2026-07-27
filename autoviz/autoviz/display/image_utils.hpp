/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <QImage>

#include <automsgs/msgs/sensor_msgs/image.pb.h>

namespace autoviz {
namespace display {

QImage imageFromProto(
    const automsgs::msgs::sensor_msgs::Image& image);

}  // namespace display
}  // namespace autoviz
