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
 * @brief Linux SocketCAN wrapper.
 */

#ifndef AUTODRIVER_HARDWARE_CAN_SOCKET_HPP_
#define AUTODRIVER_HARDWARE_CAN_SOCKET_HPP_

#include <cstdint>
#include <string>

namespace autodriver {
namespace io {

/** @brief One classical CAN frame (8-byte payload). */
struct CanFrame
{
  std::uint32_t id{0};
  bool extended{false};
  std::uint8_t dlc{0};
  std::uint8_t data[8]{};
};

/**
 * @class CanSocket
 * @brief Non-blocking SocketCAN receiver.
 */
class CanSocket
{
public:
  CanSocket();
  ~CanSocket();

  CanSocket(const CanSocket &) = delete;
  CanSocket & operator=(const CanSocket &) = delete;

  /** @brief Binds to interface such as "can0". */
  bool Open(const std::string & interface);

  void Close();

  bool IsOpen() const;

  /** @brief Reads one frame; returns false on timeout. */
  bool Read(CanFrame & frame, int timeout_ms);

  const std::string & last_error() const { return last_error_; }

private:
  int fd_{-1};
  std::string last_error_;
};

}  // namespace io
}  // namespace autodriver

#endif  // AUTODRIVER_HARDWARE_CAN_SOCKET_HPP_
