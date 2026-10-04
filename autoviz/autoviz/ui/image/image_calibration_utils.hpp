/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file image_calibration_utils.hpp
 * @brief Camera intrinsics helpers: projection, frame transforms, undistortion.
 *
 * Bridges @c sensor_msgs/CameraInfo into Autoviz image-panel math used for
 * marker projection and optional lens undistortion before display.
 *
 * @see ImagePanel
 * @see image_marker_projection.hpp
 */

#pragma once

#include <optional>

#include <QImage>
#include <QMatrix4x4>
#include <QPointF>
#include <QVector3D>

#include <automsgs/msgs/sensor_msgs/camera_info.pb.h>

namespace autoviz {
namespace image {

/**
 * @struct CameraIntrinsics
 * @brief Pinhole camera model extracted from @c CameraInfo.
 *
 * @c valid is @c false until populated by @ref intrinsicsFromCameraInfo().
 */
struct CameraIntrinsics {
  int width = 0;   /**< Image width in pixels. */
  int height = 0;  /**< Image height in pixels. */
  double fx = 0.0; /**< Focal length X (pixels). */
  double fy = 0.0; /**< Focal length Y (pixels). */
  double cx = 0.0; /**< Principal point X (pixels). */
  double cy = 0.0; /**< Principal point Y (pixels). */
  std::vector<double> distortion;  /**< Distortion coefficients (model-dependent). */
  std::string distortion_model;    /**< e.g. @c plumb_bob, @c rational_polynomial. */
  bool valid = false;              /**< @c true when fields were successfully filled. */
};

/**
 * @brief Builds @ref CameraIntrinsics from a @c CameraInfo protobuf.
 *
 * @param info Source camera calibration message.
 * @return Intrinsics with @c valid set according to whether @c K / size are usable.
 */
CameraIntrinsics intrinsicsFromCameraInfo(
    const automsgs::msgs::sensor_msgs::CameraInfo& info);

/**
 * @brief Composes fixed-frame → optical-frame transform for projection.
 *
 * Combines TF @c fixed_to_camera with the CameraInfo optical-frame convention
 * so 3D points in the fixed frame can be projected with
 * @ref projectFixedPointToPixel().
 *
 * @param info CameraInfo providing frame / optical conventions.
 * @param fixed_to_camera TF matrix from fixed frame into the camera link.
 * @return 4×4 matrix mapping fixed-frame points into the optical frame.
 */
QMatrix4x4 fixedToOpticalMatrix(
    const automsgs::msgs::sensor_msgs::CameraInfo& info,
    const QMatrix4x4& fixed_to_camera);

/**
 * @brief Projects a 3D point in the optical frame to a pixel coordinate.
 *
 * @param intrinsics Valid camera intrinsics.
 * @param optical_point Point in optical frame (Z forward).
 * @return Pixel @c (u,v) when Z > 0 and projection succeeds; @c std::nullopt otherwise.
 */
std::optional<QPointF> projectOpticalPointToPixel(const CameraIntrinsics& intrinsics,
                                                  const QVector3D& optical_point);

/**
 * @brief Projects a fixed-frame 3D point to a pixel via @c fixed_to_optical.
 *
 * @param intrinsics Valid camera intrinsics.
 * @param fixed_to_optical Transform from fixed frame into optical frame.
 * @param fixed_point Point in the fixed / world frame.
 * @return Pixel coordinate, or @c std::nullopt if behind the camera / invalid.
 * @see projectOpticalPointToPixel()
 */
std::optional<QPointF> projectFixedPointToPixel(
    const CameraIntrinsics& intrinsics, const QMatrix4x4& fixed_to_optical,
    const QVector3D& fixed_point);

/**
 * @brief Undistorts @p source using the distortion model in @p intrinsics.
 *
 * @param source Input image (same resolution as calibration when possible).
 * @param intrinsics Camera model including distortion coefficients.
 * @return Undistorted image, or a copy of @p source when undistort is unavailable.
 */
QImage undistortImage(const QImage& source, const CameraIntrinsics& intrinsics);

}  // namespace image
}  // namespace autoviz
