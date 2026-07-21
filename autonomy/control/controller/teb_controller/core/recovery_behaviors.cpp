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

#include "autonomy/control/controller/teb_controller/core/recovery_behaviors.hpp"

#include <cmath>

#include <g2o/stuff/misc.h>

namespace autonomy {
namespace control {
namespace controller {
namespace teb_controller {

void FailureDetector::update(const Twist& twist, double v_max,
                             double v_backwards_max, double omega_max,
                             double v_eps, double omega_eps) {
    if (buffer_length_ == 0) {
        return;
    }

    VelMeasurement measurement;
    measurement.v = twist.linear.x;
    measurement.omega = twist.angular.z;

    if (measurement.v > 0 && v_max > 0) {
        measurement.v /= v_max;
    } else if (measurement.v < 0 && v_backwards_max > 0) {
        measurement.v /= v_backwards_max;
    }

    if (omega_max > 0) {
        measurement.omega /= omega_max;
    }

    buffer_.push_back(measurement);
    while (buffer_.size() > buffer_length_) {
        buffer_.pop_front();
    }

    detect(v_eps, omega_eps);
}

void FailureDetector::clear() {
    buffer_.clear();
    oscillating_ = false;
}

bool FailureDetector::detect(double v_eps, double omega_eps) {
    oscillating_ = false;

    if (buffer_length_ == 0 || buffer_.size() < buffer_length_ / 2) {
        return false;
    }

    const double n = static_cast<double>(buffer_.size());
    double v_mean = 0;
    double omega_mean = 0;
    int omega_zero_crossings = 0;
    for (std::size_t i = 0; i < buffer_.size(); ++i) {
        v_mean += buffer_[i].v;
        omega_mean += buffer_[i].omega;
        if (i > 0 &&
            g2o::sign(buffer_[i].omega) != g2o::sign(buffer_[i - 1].omega)) {
            ++omega_zero_crossings;
        }
    }
    v_mean /= n;
    omega_mean /= n;

    if (std::abs(v_mean) < v_eps && std::abs(omega_mean) < omega_eps &&
        omega_zero_crossings > 1) {
        oscillating_ = true;
    }
    return oscillating_;
}

}  // namespace teb_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
