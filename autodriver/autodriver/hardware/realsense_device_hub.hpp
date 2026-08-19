/*
 * Copyright 2026 Autodriver contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file
 * @brief Shared Intel RealSense device session (one pipeline per serial).
 */

#ifndef AUTODRIVER_HARDWARE_REALSENSE_DEVICE_HUB_HPP_
#define AUTODRIVER_HARDWARE_REALSENSE_DEVICE_HUB_HPP_

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "autodriver/driver_params.hpp"

namespace autodriver {
namespace hardware {
namespace realsense {

enum class StreamKind {
    kColor,
    kDepth,
    kInfrared1,
    kInfrared2,
};

StreamKind ParseStreamKind(const std::string& text, StreamKind default_kind);
bool MatchesModelFilter(const std::string& product_name,
                        const std::string& model_filter);
std::string EncodingForStreamKind(StreamKind kind);

}  // namespace realsense
}  // namespace hardware

namespace io {

/** @brief Returns true when autodriver was built with librealsense2. */
bool RealSenseAvailable();

/**
 * @struct RealSenseVideoFrame
 * @brief Decoded video buffer emitted by RealSenseDeviceHub.
 */
struct RealSenseVideoFrame
{
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::string encoding;
  std::vector<std::uint8_t> data;
};

using RealSenseVideoCallback = std::function<void(RealSenseVideoFrame frame)>;
using RealSenseImuCallback = std::function<void(
    std::array<double, 3> linear_acceleration,
    std::array<double, 3> angular_velocity)>;

/**
 * @class RealSenseDeviceHub
 * @brief Reference-counted pipeline shared by camera/IMU drivers on one device.
 *
 * Multiple HAL drivers (color + depth + IMU on D435i) attach to the same hub
 * keyed by serial number so librealsense opens a single pipeline profile.
 */
class RealSenseDeviceHub
{
public:
  /** @brief Acquires or creates a hub for the device described in params. */
  static std::shared_ptr<RealSenseDeviceHub> Acquire(
    const hardware::DriverParams & params);

  ~RealSenseDeviceHub();

  /**
   * @brief Registers a video stream consumer.
   * @return Subscription id for Unsubscribe().
   */
  std::uint64_t SubscribeVideo(
    hardware::realsense::StreamKind stream,
    int width,
    int height,
    int fps,
    RealSenseVideoCallback callback);

  /** @brief Registers an IMU consumer (D435i and compatible models). */
  std::uint64_t SubscribeImu(RealSenseImuCallback callback);

  void Unsubscribe(std::uint64_t subscription_id);

  bool Start();
  void Stop();
  bool IsRunning() const;

  /** @brief Last error from Start() or the capture thread. */
  const std::string & last_error() const;

private:
  explicit RealSenseDeviceHub(const hardware::DriverParams & params);

  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace io
}  // namespace autodriver

#endif  // AUTODRIVER_HARDWARE_REALSENSE_DEVICE_HUB_HPP_
