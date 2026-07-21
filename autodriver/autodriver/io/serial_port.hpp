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

#ifndef AUTODRIVER_IO_SERIAL_PORT_HPP_
#define AUTODRIVER_IO_SERIAL_PORT_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

namespace autodriver {
namespace io {

/**
 * @class SerialPort
 * @brief Blocking read/write serial I/O for sensor backends.
 */
class SerialPort
{
public:
  SerialPort();
  ~SerialPort();

  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  /**
   * @brief Opens a TTY device.
   * @param device Path such as /dev/ttyUSB0.
   * @param baud_rate 9600, 115200, 460800, etc.
   */
  bool Open(const std::string & device, int baud_rate);

  void Close();

  bool IsOpen() const;

  /** @brief Reads up to max_bytes; returns bytes read (0 on timeout). */
  std::size_t Read(
    std::uint8_t * buffer,
    std::size_t max_bytes,
    int timeout_ms);

  /** @brief Writes all bytes; returns false on short write or error. */
  bool Write(const std::uint8_t * buffer, std::size_t length);

  const std::string & last_error() const { return last_error_; }

private:
  int fd_{-1};
  std::string last_error_;
};

}  // namespace io
}  // namespace autodriver

#endif  // AUTODRIVER_IO_SERIAL_PORT_HPP_
