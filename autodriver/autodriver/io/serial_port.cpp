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
 * @brief Implements SerialPort on Linux termios.
 */

#include "autodriver/io/serial_port.hpp"

#include <cerrno>
#include <cstring>
#include <string>

#if defined(__linux__)
#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace autodriver {
namespace io {
namespace {

#if defined(__linux__)
speed_t ToTermiosBaud(int baud_rate)
{
  switch (baud_rate) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default: return B115200;
  }
}
#endif

}  // namespace

SerialPort::SerialPort() = default;

SerialPort::~SerialPort()
{
  Close();
}

bool SerialPort::Open(const std::string & device, int baud_rate)
{
  Close();
#if !defined(__linux__)
  last_error_ = "SerialPort is only supported on Linux";
  return false;
#else
  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    last_error_ = std::string("open failed: ") + std::strerror(errno);
    fd_ = -1;
    return false;
  }

  termios options{};
  if (tcgetattr(fd_, &options) != 0) {
    last_error_ = std::string("tcgetattr failed: ") + std::strerror(errno);
    Close();
    return false;
  }

  cfmakeraw(&options);
  cfsetispeed(&options, ToTermiosBaud(baud_rate));
  cfsetospeed(&options, ToTermiosBaud(baud_rate));
  options.c_cflag |= (CLOCAL | CREAD);
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 0;

  if (tcsetattr(fd_, TCSANOW, &options) != 0) {
    last_error_ = std::string("tcsetattr failed: ") + std::strerror(errno);
    Close();
    return false;
  }

  return true;
#endif
}

void SerialPort::Close()
{
#if defined(__linux__)
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
#endif
}

bool SerialPort::IsOpen() const
{
  return fd_ >= 0;
}

std::size_t SerialPort::Read(
  std::uint8_t * buffer,
  std::size_t max_bytes,
  int timeout_ms)
{
#if !defined(__linux__)
  (void)buffer;
  (void)max_bytes;
  (void)timeout_ms;
  return 0;
#else
  if (fd_ < 0 || buffer == nullptr || max_bytes == 0) {
    return 0;
  }

  fd_set read_set;
  FD_ZERO(&read_set);
  FD_SET(fd_, &read_set);
  timeval tv{};
  tv.tv_sec = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms % 1000) * 1000;

  const int ready = select(fd_ + 1, &read_set, nullptr, nullptr, &tv);
  if (ready <= 0) {
    return 0;
  }

  const ssize_t n = ::read(fd_, buffer, max_bytes);
  if (n <= 0) {
    return 0;
  }
  return static_cast<std::size_t>(n);
#endif
}

bool SerialPort::Write(const std::uint8_t * buffer, std::size_t length)
{
#if !defined(__linux__)
  (void)buffer;
  (void)length;
  return false;
#else
  if (fd_ < 0 || buffer == nullptr || length == 0) {
    return false;
  }
  std::size_t written = 0;
  while (written < length) {
    const ssize_t n = ::write(fd_, buffer + written, length - written);
    if (n <= 0) {
      last_error_ = std::string("write failed: ") + std::strerror(errno);
      return false;
    }
    written += static_cast<std::size_t>(n);
  }
  return true;
#endif
}

}  // namespace io
}  // namespace autodriver
