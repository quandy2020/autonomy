/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

#pragma once

#include <automsgs/msgs/visualization_msgs/marker.pb.h>

#include "autoviz/transform/buffer.hpp"
#include "autoviz/ui/image/image_annotation_parser.hpp"
#include "autoviz/ui/image/image_calibration_utils.hpp"

namespace autoviz {
namespace image {

ImageAnnotationLayer projectMarkerToLayer(
    const automsgs::msgs::visualization_msgs::Marker& marker,
    const CameraIntrinsics& intrinsics, const QMatrix4x4& fixed_to_optical,
    const std::string& fixed_frame, autoviz::transform::Buffer* tf_buffer);

}  // namespace image
}  // namespace autoviz
