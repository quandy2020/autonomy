/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file linux_joystick.cpp
 * @brief LinuxJoystick implementation (linux/joystick.h).
 */

#include "autodriver/joy/linux_joystick.hpp"

#include <cerrno>
#include <cstring>

#include "autolink/common/log.hpp"

#if defined(__linux__)
#include <fcntl.h>
#include <linux/joystick.h>
#include <sys/ioctl.h>
#include <unistd.h>
#endif

namespace autodriver {
namespace joy {
namespace {

constexpr float kAxisScale = 32767.0f;

}  // namespace

LinuxJoystick::~LinuxJoystick() { Close(); }

bool LinuxJoystick::Open(const std::string& device_path) {
  Close();
#if !defined(__linux__)
  AERROR << "LinuxJoystick: joystick API is only available on Linux";
  (void)device_path;
  return false;
#else
  const int fd = ::open(device_path.c_str(), O_RDONLY | O_NONBLOCK);
  if (fd < 0) {
    AERROR << "LinuxJoystick: open failed path=" << device_path
           << " errno=" << errno << " (" << std::strerror(errno) << ")";
    return false;
  }
  fd_ = fd;
  path_ = device_path;

  __u8 axis_count = 0;
  __u8 button_count = 0;
  if (::ioctl(fd_, JSIOCGAXES, &axis_count) == 0) {
    axes_.assign(static_cast<std::size_t>(axis_count), 0.0f);
  }
  if (::ioctl(fd_, JSIOCGBUTTONS, &button_count) == 0) {
    buttons_.assign(static_cast<std::size_t>(button_count), 0);
  }
  AINFO << "LinuxJoystick opened path=" << path_
        << " axes=" << axes_.size() << " buttons=" << buttons_.size();
  return true;
#endif
}

void LinuxJoystick::Close() {
#if defined(__linux__)
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
#else
  fd_ = -1;
#endif
  path_.clear();
  std::lock_guard<std::mutex> lock(mutex_);
  axes_.clear();
  buttons_.clear();
}

bool LinuxJoystick::IsOpen() const { return fd_ >= 0; }

int LinuxJoystick::Poll() {
#if !defined(__linux__)
  return 0;
#else
  if (fd_ < 0) {
    return 0;
  }
  int count = 0;
  js_event event{};
  while (true) {
    const ssize_t n = ::read(fd_, &event, sizeof(event));
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        break;
      }
      AERROR << "LinuxJoystick: read failed errno=" << errno;
      Close();
      break;
    }
    if (n != static_cast<ssize_t>(sizeof(event))) {
      break;
    }
    const uint8_t type = event.type & ~JS_EVENT_INIT;
    std::lock_guard<std::mutex> lock(mutex_);
    if (type == JS_EVENT_AXIS) {
      EnsureAxis(event.number);
      axes_[event.number] =
          static_cast<float>(event.value) / kAxisScale;
      ++count;
    } else if (type == JS_EVENT_BUTTON) {
      EnsureButton(event.number);
      buttons_[event.number] = event.value ? 1 : 0;
      ++count;
    }
  }
  return count;
#endif
}

std::size_t LinuxJoystick::AxisCount() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return axes_.size();
}

std::size_t LinuxJoystick::ButtonCount() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return buttons_.size();
}

float LinuxJoystick::Axis(std::size_t index) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (index >= axes_.size()) {
    return 0.0f;
  }
  return axes_[index];
}

int LinuxJoystick::Button(std::size_t index) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (index >= buttons_.size()) {
    return 0;
  }
  return buttons_[index];
}

std::vector<float> LinuxJoystick::Axes() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return axes_;
}

std::vector<int32_t> LinuxJoystick::Buttons() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return buttons_;
}

void LinuxJoystick::EnsureAxis(std::size_t index) {
  if (index >= axes_.size()) {
    axes_.resize(index + 1, 0.0f);
  }
}

void LinuxJoystick::EnsureButton(std::size_t index) {
  if (index >= buttons_.size()) {
    buttons_.resize(index + 1, 0);
  }
}

}  // namespace joy
}  // namespace autodriver
