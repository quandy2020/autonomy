/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
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
 * @file can_socket.hpp
 * @brief Linux SocketCAN wrapper.
 */

#ifndef AUTODRIVER_COMMON_CAN_SOCKET_HPP_
#define AUTODRIVER_COMMON_CAN_SOCKET_HPP_

#include <cstdint>
#include <string>
#include "autolink/common/macros.hpp"

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
   * @brief SharedPtr / ConstSharedPtr aliases and Class::make_shared().
   */
  AUTOLINK_SHARED_PTR_DEFINITIONS(CanSocket)

  /**
   * @brief Disable copy construction and copy assignment.
   */
  DISALLOW_COPY_AND_ASSIGN(CanSocket)

  /**
   * @brief Default-construct a closed SocketCAN wrapper.
   */
  CanSocket();

  /**
   * @brief Closes the socket if still open.
   */
  ~CanSocket();



  /**
   * @brief Opens a raw SocketCAN interface for reading frames.
   * @param[in] interface Interface name (e.g. "can0").
   * @return true when the socket was opened successfully; false on failure.
   */
  bool Open(const std::string & interface);

  /**
   * @brief Closes the underlying CAN socket descriptor.
   */
  void Close();

  /**
   * @brief Whether the CAN socket is currently open.
   * @return true when a valid file descriptor is held.
   */
  bool IsOpen() const;

  /**
   * @brief Reads one CAN frame, waiting up to timeout_ms milliseconds.
   * @param[out] frame Destination classical CAN frame.
   * @param[in] timeout_ms Poll/select timeout in milliseconds.
   * @return true when a frame was read; false on timeout or error.
   */
  bool Read(CanFrame & frame, int timeout_ms);

  /**
   * @brief Writes one CAN frame (SocketCAN send).
   * @param[in] frame Classical CAN frame to transmit.
   * @return true when the frame was sent successfully; false on error.
   */
  bool Write(const CanFrame & frame);

  /**
   * @brief Last error message from Open() or Read().
   * @return Human-readable error text.
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
