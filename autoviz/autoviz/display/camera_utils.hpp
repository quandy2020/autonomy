/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file camera_utils.hpp
 * @brief Camera frustum / far-plane geometry and optical-frame transforms for
 *        Camera and CameraInfo displays.
 *
 * Builds wireframe frustum segments and a textured far-plane quad in the ROS
 * optical frame (X right, Y down, Z forward), plus matrices that convert
 * optical coordinates into @c camera_link / fixed-frame poses.
 *
 * @see CameraDisplay
 * @see CameraInfoDisplay
 * @see buildCameraFrustumSegments
 */

#pragma once

#include <vector>

#include <QMatrix4x4>
#include <QVector3D>

#include <automsgs/msgs/sensor_msgs/camera_info.pb.h>

namespace autoviz {
namespace display {

/**
 * @struct CameraFarPlane
 * @brief Rectangle corners of the camera far plane in optical coordinates.
 *
 * Used to project a camera image onto a billboard at @c far_distance for
 * @ref CameraDisplay. When @c valid is @c false, intrinsics were insufficient
 * to build the plane.
 *
 * @see buildCameraFarPlane
 */
struct CameraFarPlane {
  QVector3D top_left;      /**< Far-plane corner (optical): top-left. */
  QVector3D top_right;     /**< Far-plane corner (optical): top-right. */
  QVector3D bottom_right;  /**< Far-plane corner (optical): bottom-right. */
  QVector3D bottom_left;   /**< Far-plane corner (optical): bottom-left. */
  bool valid = false;      /**< @c true when corners were computed successfully. */
};

/**
 * @brief Builds camera frustum edge segments in the optical frame.
 *
 * ROS optical convention: X right, Y down, Z forward. Returns near/far
 * rectangle edges plus the four rays from the optical origin.
 *
 * @param info Camera intrinsics (@c K, image size).
 * @param near_distance Near clip distance along Z (meters).
 * @param far_distance Far clip distance along Z (meters).
 * @return List of line segments as @c (start, end) pairs in optical frame.
 *
 * @see buildCameraFarPlane
 * @see CameraInfoDisplay
 */
std::vector<std::pair<QVector3D, QVector3D>> buildCameraFrustumSegments(
    const automsgs::msgs::sensor_msgs::CameraInfo& info,
    float near_distance, float far_distance);

/**
 * @brief Computes far-plane rectangle corners in the optical frame for
 *        texture projection.
 *
 * @param info Camera intrinsics.
 * @param far_distance Distance of the plane along optical Z (meters).
 * @return @ref CameraFarPlane with @c valid set on success.
 *
 * @see buildCameraFrustumSegments
 * @see CameraDisplay
 */
CameraFarPlane buildCameraFarPlane(
    const automsgs::msgs::sensor_msgs::CameraInfo& info,
    float far_distance);

/**
 * @brief Fixed transform from OpenCV optical axes to REP-103 @c camera_link.
 *
 * Optical: X right, Y down, Z forward → camera_link: X forward, Y left,
 * Z up (standard ROS camera_link).
 *
 * @return Constant @c QMatrix4x4 for the optical→camera_link basis change.
 *
 * @see opticalToWorld
 */
QMatrix4x4 opticalToCameraLinkMatrix();

/**
 * @brief Maps optical-frame points into the fixed/world frame.
 *
 * Composes @ref opticalToCameraLinkMatrix with @p camera_to_fixed (pose of
 * the camera_link / header frame in fixed frame).
 *
 * @param info CameraInfo (frame id used indirectly via @p camera_to_fixed).
 * @param camera_to_fixed Transform from camera frame to fixed frame.
 * @return Optical → fixed/world matrix.
 *
 * @see opticalToCameraLinkMatrix
 */
QMatrix4x4 opticalToWorld(
    const automsgs::msgs::sensor_msgs::CameraInfo& info,
    const QMatrix4x4& camera_to_fixed);

}  // namespace display
}  // namespace autoviz
