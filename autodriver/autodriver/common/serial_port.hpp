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
 * @file serial_port.hpp
 * @brief Linux serial port wrapper (termios).
 */

#ifndef AUTODRIVER_COMMON_SERIAL_PORT_HPP_
#define AUTODRIVER_COMMON_SERIAL_PORT_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include "autolink/common/macros.hpp"

namespace autodriver {
namespace io {

/**
 * @class autodriver::io::SerialPort
 * @brief Blocking read/write serial I/O for sensor backends.
 */
class SerialPort
{
public:
  /**
   * @brief SharedPtr / ConstSharedPtr aliases and Class::make_shared().
   */
  AUTOLINK_SHARED_PTR_DEFINITIONS(SerialPort)

  /**
   * @brief Disable copy construction and copy assignment.
   */
  DISALLOW_COPY_AND_ASSIGN(SerialPort)

  /**
   * @brief Default-construct a closed serial port.
   */
  SerialPort();

  /**
   * @brief Closes the serial port if still open.
   */
  ~SerialPort();



  /**
   * @brief Opens a TTY device in raw 8N1 mode at the given baud rate.
   * @param[in] device TTY path (e.g. "/dev/ttyUSB0").
   * @param[in] baud_rate Desired baud rate.
   * @return true when the port was opened successfully; false on failure.
   */
  bool Open(const std::string & device, int baud_rate);

  /**
   * @brief Closes the underlying serial file descriptor.
   */
  void Close();

  /**
   * @brief Whether the serial port is currently open.
   * @return true when a valid file descriptor is held.
   */
  bool IsOpen() const;

  /**
   * @brief Reads up to max_bytes with a select-based timeout.
   * @param[out] buffer Destination buffer for received bytes.
   * @param[in] max_bytes Capacity of @p buffer.
   * @param[in] timeout_ms Select timeout in milliseconds.
   * @return Number of bytes read; 0 on timeout without error.
   */
  std::size_t Read(
    std::uint8_t * buffer,
    std::size_t max_bytes,
    int timeout_ms);

  /**
   * @brief Writes length bytes to the serial port, retrying partial writes.
   * @param[out] buffer Source bytes to write.
   * @param[in] length Number of bytes to write.
   * @return true when all bytes were written; false on I/O error.
   */
  bool Write(const std::uint8_t * buffer, std::size_t length);

  /**
   * @brief Last error message from Open(), Read(), or Write().
   * @return Human-readable error text.
   */
  const std::string & last_error() const { return last_error_; }

private:
  // Open file descriptor, or -1 when closed.
  int fd_{-1};

  // Last reported I/O error message.
  std::string last_error_;
};

}  // namespace io
}  // namespace autodriver

#endif  // AUTODRIVER_COMMON_SERIAL_PORT_HPP_
