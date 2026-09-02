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
 * @brief Implements CanSocket on Linux SocketCAN.
 */

#include "autodriver/hardware/can_socket.hpp"

#include <cstring>
#include <string>

#if defined(__linux__)
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace autodriver {
namespace io {

CanSocket::CanSocket() = default;

/** @brief Closes the socket if still open. */
CanSocket::~CanSocket()
{
  Close();
}

/** @brief Opens a raw SocketCAN interface for reading frames. */
bool CanSocket::Open(const std::string & interface)
{
  Close();
#if !defined(__linux__)
  last_error_ = "CanSocket is only supported on Linux";
  return false;
#else
  fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd_ < 0) {
    last_error_ = "socket(PF_CAN) failed";
    fd_ = -1;
    return false;
  }

  ifreq ifr{};
  std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
  if (ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
    last_error_ = "SIOCGIFINDEX failed for " + interface;
    Close();
    return false;
  }

  sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    last_error_ = "bind failed for " + interface;
    Close();
    return false;
  }

  return true;
#endif
}

/** @brief Closes the underlying CAN socket descriptor. */
void CanSocket::Close()
{
#if defined(__linux__)
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
#endif
}

/** @brief Returns true when the CAN socket is open. */
bool CanSocket::IsOpen() const
{
  return fd_ >= 0;
}

/** @brief Reads one CAN frame, waiting up to timeout_ms milliseconds. */
bool CanSocket::Read(CanFrame & frame, int timeout_ms)
{
#if !defined(__linux__)
  (void)frame;
  (void)timeout_ms;
  return false;
#else
  if (fd_ < 0) {
    return false;
  }

  fd_set read_set;
  FD_ZERO(&read_set);
  FD_SET(fd_, &read_set);
  timeval tv{};
  tv.tv_sec = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms % 1000) * 1000;

  const int ready = select(fd_ + 1, &read_set, nullptr, nullptr, &tv);
  if (ready <= 0) {
    return false;
  }

  can_frame raw{};
  const ssize_t n = ::read(fd_, &raw, sizeof(raw));
  if (n != static_cast<ssize_t>(sizeof(raw))) {
    return false;
  }

  frame.extended = (raw.can_id & CAN_EFF_FLAG) != 0;
  frame.id = raw.can_id & (frame.extended ? CAN_EFF_MASK : CAN_SFF_MASK);
  frame.dlc = raw.can_dlc;
  for (int i = 0; i < 8; ++i) {
    frame.data[i] = raw.data[i];
  }
  return true;
#endif
}

}  // namespace io
}  // namespace autodriver
