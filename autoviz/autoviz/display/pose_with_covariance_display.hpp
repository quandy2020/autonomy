/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file pose_with_covariance_display.hpp
 * @brief Channel display for @c geometry_msgs/PoseWithCovarianceStamped.
 *
 * Draws the mean pose (arrow/axes) plus an optional RViz CovarianceVisual
 * for the 6×6 covariance — aligned with RViz2 PoseWithCovariance.
 *
 * ## Properties
 *
 * - @c color / @c axis_length — mean pose appearance
 * - @c covariance_scale / @c orientation_covariance_scale /
 *   @c orientation_covariance_offset — ellipsoid / orientation visual scales
 * - @c show_covariance — toggle covariance drawing
 *
 * @see ChannelDisplay
 * @see PoseDisplay
 * @see drawCovarianceOgreOrGl()
 */

#pragma once

#include <array>

#include <QQuaternion>
#include <QVector3D>

#include <automsgs/msgs/geometry_msgs/pose_with_covariance_stamped.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/channel_display.hpp"

namespace autoviz {
namespace display {

/**
 * @class PoseWithCovarianceDisplay
 * @brief Subscribes to PoseWithCovarianceStamped and draws pose + covariance.
 *
 * ## Data flow
 *
 * - **In:** @ref processMessage() TF-transforms pose/orientation and copies
 *   the 36-element covariance.
 * - **Out:** @ref onDraw() draws the mean pose and, when enabled,
 *   @ref drawCovarianceOgreOrGl.
 */
class PoseWithCovarianceDisplay
    : public ChannelDisplay<
          automsgs::msgs::geometry_msgs::PoseWithCovarianceStamped> {
 public:
  /**
   * @brief Constructs a PoseWithCovariance display bound to @p channel.
   *
   * @param channel Autolink / topic name for PoseWithCovarianceStamped.
   */
  explicit PoseWithCovarianceDisplay(std::string channel);

  /**
   * @brief Catalog type id used by the display registry.
   * @return Always @c "PoseWithCovariance".
   */
  std::string typeId() const override { return "PoseWithCovariance"; }

  /**
   * @brief Declares pose color/size and covariance scale properties.
   * @return Property specification list for the Displays tree.
   */
  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  /**
   * @brief Caches pose, orientations, and covariance from @p message.
   *
   * @param message Incoming PoseWithCovarianceStamped.
   */
  void processMessage(
      const automsgs::msgs::geometry_msgs::PoseWithCovarianceStamped& message)
      override;

  /**
   * @brief Clears @c have_pose_ (Time-panel Reset).
   */
  void clearReceivedData() override;

  /**
   * @brief Draws mean pose and optional covariance into @p scene.
   *
   * @param scene Scene overlay / Ogre host for the current frame.
   */
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  bool have_pose_ = false;                 /**< Whether cached pose is valid. */
  QVector3D position_;                     /**< Mean position (fixed frame). */
  QQuaternion pose_orientation_;           /**< Pose orientation. */
  QQuaternion frame_orientation_;          /**< Parent-frame orientation. */
  float yaw_ = 0.f;                        /**< Yaw (rad) for arrow/axes. */
  std::array<double, 36> covariance_{};    /**< Row-major 6×6 covariance. */
};

}  // namespace display
}  // namespace autoviz
