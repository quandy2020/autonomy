/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file depth_cloud_utils.hpp
 * @brief Projects a depth @c sensor_msgs/Image into colored 3D points using
 *        CameraInfo intrinsics.
 *
 * Shared by @ref DepthCloudDisplay. Points are returned in the camera optical
 * frame (X right, Y down, Z forward); the display transforms them to fixed
 * frame at draw time.
 *
 * @see DepthCloudDisplay
 * @see projectDepthImage
 */

#pragma once

#include <vector>

#include <QColor>
#include <QImage>
#include <QVector3D>

#include <automsgs/msgs/sensor_msgs/camera_info.pb.h>
#include <automsgs/msgs/sensor_msgs/image.pb.h>

namespace autoviz {
namespace display {

/**
 * @struct DepthCloudPoint
 * @brief One projected depth sample with display color.
 *
 * @see projectDepthImage
 */
struct DepthCloudPoint {
  QVector3D position;  /**< Point in camera optical frame (meters). */
  QColor color;        /**< Per-point RGBA for overlay drawing. */
};

/**
 * @brief Projects a depth image into optical-frame colored points.
 *
 * Uses @p info intrinsics to back-project each (possibly decimated) pixel.
 * When @p color_image is non-null and sized appropriately, RGB is sampled
 * from it; otherwise @p flat_color is used for every point.
 *
 * @param depth Depth image protobuf (encoding handled in .cpp).
 * @param info Matching CameraInfo for @c fx,@c fy,@c cx,@c cy.
 * @param color_image Optional aligned color image; may be @c nullptr.
 * @param decimation Keep every N-th pixel along rows/cols (≥ 1).
 * @param flat_color Fallback color when no color image is available.
 * @return Optical-frame points suitable for SceneOverlay point drawing.
 *
 * @note Invalid / NaN / out-of-range depths are skipped.
 *
 * @see DepthCloudDisplay
 * @see DepthCloudPoint
 */
std::vector<DepthCloudPoint> projectDepthImage(
    const automsgs::msgs::sensor_msgs::Image& depth,
    const automsgs::msgs::sensor_msgs::CameraInfo& info,
    const QImage* color_image, uint32_t decimation, const QColor& flat_color);

}  // namespace display
}  // namespace autoviz
