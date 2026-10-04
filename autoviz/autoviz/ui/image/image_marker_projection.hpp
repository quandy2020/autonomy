/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file image_marker_projection.hpp
 * @brief Project @c visualization_msgs/Marker into image annotation layers.
 *
 * Uses camera intrinsics and fixed→optical TF to paint 3D markers as 2D
 * polylines / points / text on the Image panel.
 *
 * @see ImageAnnotationLayer
 * @see CameraIntrinsics
 * @see ImagePanel
 */

#pragma once

#include <automsgs/msgs/visualization_msgs/marker.pb.h>

#include "autoviz/transform/buffer.hpp"
#include "autoviz/ui/image/image_annotation_parser.hpp"
#include "autoviz/ui/image/image_calibration_utils.hpp"

namespace autoviz {
namespace image {

/**
 * @brief Projects a single Marker into pixel-space annotation primitives.
 *
 * Looks up the marker's frame via @p tf_buffer relative to @p fixed_frame,
 * then projects geometry with @p intrinsics and @p fixed_to_optical.
 *
 * @param marker Source visualization marker (points, lines, text, …).
 * @param intrinsics Valid camera intrinsics for the image being shown.
 * @param fixed_to_optical Fixed-frame → optical-frame transform.
 * @param fixed_frame Name of the fixed / world frame used for TF lookups.
 * @param tf_buffer Non-owning TF buffer; may be @c nullptr (projection fails).
 * @return Annotation layer (possibly empty if TF / projection fails).
 * @see projectFixedPointToPixel()
 */
ImageAnnotationLayer projectMarkerToLayer(
    const automsgs::msgs::visualization_msgs::Marker& marker,
    const CameraIntrinsics& intrinsics, const QMatrix4x4& fixed_to_optical,
    const std::string& fixed_frame, autoviz::transform::Buffer* tf_buffer);

}  // namespace image
}  // namespace autoviz
