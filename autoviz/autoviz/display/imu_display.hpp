/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file imu_display.hpp
 * @brief Displays @c sensor_msgs/Imu orientation axes plus linear accel and
 *        angular velocity arrows (RViz Imu).
 *
 * Caches orientation (roll/pitch/yaw) and accel/gyro vectors from the latest
 * message; @ref onDraw emits axes and scaled arrows.
 *
 * @see ChannelDisplay
 * @see AccelStampedDisplay
 * @see arrow_mesh_utils.hpp
 */

#pragma once

#include <QVector3D>

#include <automsgs/msgs/sensor_msgs/imu.pb.h>
#include "autoviz/common/display_property.hpp"
#include "autoviz/display/channel_display.hpp"

namespace autoviz {
namespace display {

/**
 * @class ImuDisplay
 * @brief Channel display for IMU orientation and accel/gyro arrows.
 *
 * Properties typically include accel/gyro colors, axis length, and vector
 * scales.
 *
 * @see AccelStampedDisplay
 * @see ChannelDisplay
 */
class ImuDisplay : public ChannelDisplay<automsgs::msgs::sensor_msgs::Imu> {
 public:
  /**
   * @brief Constructs the display bound to @p channel.
   *
   * @param channel Autolink channel for Imu messages.
   */
  explicit ImuDisplay(std::string channel);

  /**
   * @brief Catalog type id.
   *
   * @return Always @c "Imu".
   */
  std::string typeId() const override { return "Imu"; }

  /**
   * @brief Property schema (colors, axis length, scales, …).
   *
   * @return Specs for the Displays property editor.
   */
  std::vector<common::DisplayPropertySpec> propertySpecs() const override;

 protected:
  /**
   * @brief Caches orientation and accel/gyro from @p message.
   *
   * @param message sensor_msgs/Imu protobuf.
   */
  void processMessage(const automsgs::msgs::sensor_msgs::Imu& message) override;

  /**
   * @brief Clears cached IMU state and @c have_imu_.
   */
  void clearReceivedData() override;

  /**
   * @brief Draws orientation axes and accel/gyro arrows when data is valid.
   *
   * @param scene Scene overlay destination.
   */
  void onDraw(rendering::SceneOverlay& scene) override;

 private:
  /** @c true after a successful @ref processMessage since reset. */
  bool have_imu_ = false;

  /** IMU frame origin in fixed frame. */
  QVector3D origin_;

  /** Linear acceleration vector. */
  QVector3D linear_accel_;

  /** Angular velocity vector. */
  QVector3D angular_vel_;

  float roll_ = 0.f;   /**< Orientation roll (radians). */
  float pitch_ = 0.f;  /**< Orientation pitch (radians). */
  float yaw_ = 0.f;    /**< Orientation yaw (radians). */
};

}  // namespace display
}  // namespace autoviz
