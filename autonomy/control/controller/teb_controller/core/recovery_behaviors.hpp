/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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
 *
 * Ported from teb_local_planner recovery_behaviors (TU Dortmund).
 */

#pragma once

#include <cstddef>
#include <deque>

#include "autonomy/control/controller/teb_controller/core/teb_core.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace teb_controller {

/**
 * @brief Detect oscillation / stuck patterns from recent velocity commands.
 */
class FailureDetector {
 public:
  FailureDetector() = default;

  void setBufferLength(std::size_t length) {
    buffer_length_ = length;
    while (buffer_.size() > buffer_length_) {
      buffer_.pop_front();
    }
  }

  void update(const Twist& twist, double v_max, double v_backwards_max,
              double omega_max, double v_eps, double omega_eps);

  bool isOscillating() const { return oscillating_; }

  void clear();

 protected:
  struct VelMeasurement {
    double v = 0;
    double omega = 0;
  };

  bool detect(double v_eps, double omega_eps);

 private:
  std::deque<VelMeasurement> buffer_;
  std::size_t buffer_length_{0};
  bool oscillating_{false};
};

}  // namespace teb_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
