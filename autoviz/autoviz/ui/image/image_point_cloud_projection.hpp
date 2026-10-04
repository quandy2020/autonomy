/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file image_point_cloud_projection.hpp
 * @brief Project sensor_msgs/PointCloud2 into image-pixel annotation points.
 */

#pragma once

#include <string>

#include <QMatrix4x4>

#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>

#include "autoviz/transform/buffer.hpp"
#include "autoviz/ui/image/image_annotation_parser.hpp"
#include "autoviz/ui/image/image_calibration_utils.hpp"

namespace autoviz {
namespace image {

/**
 * @brief Projects a PointCloud2 into drawable image points.
 *
 * Points are transformed from the cloud @c frame_id into the camera optical
 * frame via TF, then projected with @p intrinsics. Intensity (when present)
 * maps to a turbo-like color; otherwise depth along Z is used.
 *
 * @param cloud Source PointCloud2.
 * @param intrinsics Valid camera intrinsics.
 * @param fixed_to_optical Fixed → optical matrix used with TF lookups.
 * @param fixed_frame Fixed / world frame name for TF.
 * @param tf_buffer TF buffer (non-owning).
 * @param max_points Decimation cap for interactive frame rates.
 * @return Annotation layer of projected points (may be empty).
 */
ImageAnnotationLayer projectPointCloudToLayer(
    const automsgs::msgs::sensor_msgs::PointCloud2& cloud,
    const CameraIntrinsics& intrinsics, const QMatrix4x4& fixed_to_optical,
    const std::string& fixed_frame, autoviz::transform::Buffer* tf_buffer,
    int max_points = 40000);

}  // namespace image
}  // namespace autoviz
