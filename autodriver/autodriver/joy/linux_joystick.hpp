/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file linux_joystick.hpp
 * @brief Thin wrapper around Linux joystick device (/dev/input/js*).
 */

#ifndef AUTODRIVER_JOY_LINUX_JOYSTICK_HPP_
#define AUTODRIVER_JOY_LINUX_JOYSTICK_HPP_

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace autodriver {
namespace joy {

/**
 * @class autodriver::joy::LinuxJoystick
 * @brief Opens a joystick device and maintains axes/buttons from js_event.
 *
 * Linux only. On other platforms Open() fails and IsOpen() stays false.
 */
class LinuxJoystick {
 public:
  LinuxJoystick() = default;
  ~LinuxJoystick();

  LinuxJoystick(const LinuxJoystick&) = delete;
  LinuxJoystick& operator=(const LinuxJoystick&) = delete;

  /**
   * @brief Open @p device_path (e.g. /dev/input/js0).
   * @return true on success.
   */
  bool Open(const std::string& device_path);

  /** @brief Close the device if open. */
  void Close();

  /** @return true when a valid fd is held. */
  bool IsOpen() const;

  /**
   * @brief Drain pending events (non-blocking).
   * @return Number of events consumed this call.
   */
  int Poll();

  /** @brief Axis count reported by the driver (may grow as events arrive). */
  std::size_t AxisCount() const;

  /** @brief Button count reported by the driver. */
  std::size_t ButtonCount() const;

  /**
   * @brief Axis value in [-1, 1] (raw int16 scaled).
   * @param[in] index Axis index.
   * @return 0 when index is out of range.
   */
  float Axis(std::size_t index) const;

  /**
   * @brief Button state (0 or 1).
   * @param[in] index Button index.
   * @return 0 when index is out of range.
   */
  int Button(std::size_t index) const;

  /** @brief Snapshot of all axes. */
  std::vector<float> Axes() const;

  /** @brief Snapshot of all buttons. */
  std::vector<int32_t> Buttons() const;

  /** @brief Device path last passed to Open(). */
  const std::string& Path() const { return path_; }

 private:
  void EnsureAxis(std::size_t index);
  void EnsureButton(std::size_t index);

  int fd_ = -1;
  std::string path_;
  mutable std::mutex mutex_;
  std::vector<float> axes_;
  std::vector<int32_t> buttons_;
};

}  // namespace joy
}  // namespace autodriver

#endif  // AUTODRIVER_JOY_LINUX_JOYSTICK_HPP_
