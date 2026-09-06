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

#ifndef AUTODRIVER_COMMON_CAN_SOCKET_HPP_
#define AUTODRIVER_COMMON_CAN_SOCKET_HPP_

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
   * @brief Copy constructor (deleted)
   */
  CanSocket();

  /**
   * @brief Closes the socket if still open.
   */
  ~CanSocket();

  CanSocket(const CanSocket &) = delete;

  /**
   * @brief Copy assignment operator (deleted)
   */
  CanSocket & operator=(const CanSocket &) = delete;

  /**
   * @brief Opens a raw SocketCAN interface for reading frames.
   */
  bool Open(const std::string & interface);

  /**
   * @brief Closes the underlying CAN socket descriptor.
   */
  void Close();

  /**
   * @brief Returns true when the CAN socket is open.
   */
  bool IsOpen() const;

  /**
   * @brief Reads one CAN frame, waiting up to timeout_ms milliseconds.
   */
  bool Read(CanFrame & frame, int timeout_ms);

  /**
   * @brief Writes one CAN frame (SocketCAN send).
   */
  bool Write(const CanFrame & frame);

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

#endif  // AUTODRIVER_COMMON_CAN_SOCKET_HPP_
