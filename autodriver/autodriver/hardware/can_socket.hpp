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

/**
 * @brief One classical CAN frame (8-byte payload).
 */
struct CanFrame
{
  // CAN identifier (11- or 29-bit depending on extended).
  std::uint32_t id{0};

  // True for 29-bit extended identifiers.
  bool extended{false};

  // Data length code (0-8).
  std::uint8_t dlc{0};

  // Payload bytes (only first dlc entries are valid).
  std::uint8_t data[8]{};
};

/**
 * @class autodriver::io::CanSocket
 * @brief Non-blocking SocketCAN receiver.
 */
class CanSocket
{
public:
  /**
   * @brief Constructor for autodriver::io::CanSocket
   */
  CanSocket();

  /**
   * @brief Destructor for autodriver::io::CanSocket
   */
  ~CanSocket();

  /**
   * @brief Copy constructor (deleted)
   */
  CanSocket(const CanSocket &) = delete;

  /**
   * @brief Copy assignment operator (deleted)
   */
  CanSocket & operator=(const CanSocket &) = delete;

  /**
   * @brief Binds to interface such as "can0".
   * @param interface Linux CAN network interface name
   * @return True on success
   */
  bool Open(const std::string & interface);

  /**
   * @brief Close the bound CAN socket.
   */
  void Close();

  /**
   * @brief Whether a CAN socket is currently open.
   * @return True when Open() succeeded and Close() has not been called
   */
  bool IsOpen() const;

  /**
   * @brief Read one frame from the socket.
   * @param frame Output frame structure
   * @param timeout_ms Read timeout in milliseconds
   * @return False on timeout or error
   */
  bool Read(CanFrame & frame, int timeout_ms);

  /**
   * @brief Last error message from Open() or Read().
   * @return Human-readable error text
   */
  const std::string & last_error() const { return last_error_; }

private:
  // Open socket file descriptor, or -1 when closed.
  int fd_{-1};

  // Last reported I/O error message.
  std::string last_error_;
};

}  // namespace io
}  // namespace autodriver

#endif  // AUTODRIVER_HARDWARE_CAN_SOCKET_HPP_
