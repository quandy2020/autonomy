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
 * @brief Linux serial port wrapper (termios).
 */

#ifndef AUTODRIVER_COMMON_SERIAL_PORT_HPP_
#define AUTODRIVER_COMMON_SERIAL_PORT_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

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
   * @brief Copy constructor (deleted)
   */
  SerialPort();

  /**
   * @brief Closes the serial port if still open.
   */
  ~SerialPort();

  SerialPort(const SerialPort &) = delete;

  /**
   * @brief Copy assignment operator (deleted)
   */
  SerialPort & operator=(const SerialPort &) = delete;

  /**
   * @brief Opens a TTY device in raw 8N1 mode at the given baud rate.
   */
  bool Open(const std::string & device, int baud_rate);

  /**
   * @brief Closes the underlying serial file descriptor.
   */
  void Close();

  /**
   * @brief Returns true when the serial port is open.
   */
  bool IsOpen() const;

  /**
   * @brief Reads up to max_bytes with a select-based timeout.
   */
  std::size_t Read(
    std::uint8_t * buffer,
    std::size_t max_bytes,
    int timeout_ms);

  /**
   * @brief Writes length bytes to the serial port, retrying partial writes.
   */
  bool Write(const std::uint8_t * buffer, std::size_t length);

  /**
   * @brief Last error message from Open(), Read(), or Write().
   * @return Human-readable error text
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
