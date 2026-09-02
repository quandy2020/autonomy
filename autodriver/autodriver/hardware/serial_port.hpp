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

#ifndef AUTODRIVER_HARDWARE_SERIAL_PORT_HPP_
#define AUTODRIVER_HARDWARE_SERIAL_PORT_HPP_

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
   * @brief Constructor for autodriver::io::SerialPort
   */
  SerialPort();

  /**
   * @brief Destructor for autodriver::io::SerialPort
   */
  ~SerialPort();

  /**
   * @brief Copy constructor (deleted)
   */
  SerialPort(const SerialPort &) = delete;

  /**
   * @brief Copy assignment operator (deleted)
   */
  SerialPort & operator=(const SerialPort &) = delete;

  /**
   * @brief Opens a TTY device.
   * @param device Path such as /dev/ttyUSB0.
   * @param baud_rate 9600, 115200, 460800, etc.
   * @return True on success
   */
  bool Open(const std::string & device, int baud_rate);

  /**
   * @brief Close the open TTY device.
   */
  void Close();

  /**
   * @brief Whether a TTY device is currently open.
   * @return True when Open() succeeded and Close() has not been called
   */
  bool IsOpen() const;

  /**
   * @brief Read up to max_bytes from the port.
   * @param buffer Destination buffer
   * @param max_bytes Maximum bytes to read
   * @param timeout_ms Read timeout in milliseconds
   * @return Bytes read (0 on timeout)
   */
  std::size_t Read(
    std::uint8_t * buffer,
    std::size_t max_bytes,
    int timeout_ms);

  /**
   * @brief Write all bytes to the port.
   * @param buffer Source buffer
   * @param length Number of bytes to write
   * @return False on short write or error
   */
  bool Write(const std::uint8_t * buffer, std::size_t length);

  /**
   * @brief Last error message from Open(), Read(), or Write().
   * @return Human-readable error text
   */
  const std::string & last_error() const { return last_error_; }

private:
  /** @brief Open file descriptor, or -1 when closed. */
  int fd_{-1};

  /** @brief Last reported I/O error message. */
  std::string last_error_;
};

}  // namespace io
}  // namespace autodriver

#endif  // AUTODRIVER_HARDWARE_SERIAL_PORT_HPP_
