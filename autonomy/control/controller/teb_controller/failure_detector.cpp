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
 */

#include "autonomy/control/controller/teb_controller/failure_detector.hpp"
#include "autonomy/control/controller/teb_controller/pose2d_utils.hpp"

#include <cmath>

namespace autonomy {
namespace control {
namespace teb_controller {

// ============== FailureDetector Implementation ===================

void FailureDetector::Update(
    const autonomy::commsgs::geometry_msgs::Twist& twist, double v_max,
    double v_backwards_max, double omega_max, double v_eps, double omega_eps) {
    if (buffer_.capacity() == 0)
        return;

    commsgs::geometry_msgs::Twist2D measurement{};
    measurement.x = static_cast<float>(twist.linear.x);
    measurement.y = static_cast<float>(twist.linear.y);
    measurement.theta = static_cast<float>(twist.angular.z);

    measurement.x /= static_cast<float>(
        measurement.x > 0 && v_max > 0
            ? v_max
            : measurement.x < 0 && v_backwards_max > 0 ? v_backwards_max : 1.0);

    if (omega_max > 0) {
        measurement.theta /= static_cast<float>(omega_max);
    }

    buffer_.push_back(measurement);

    // immediately compute new state
    Detect(v_eps, omega_eps);
}

void FailureDetector::Clear() {
    buffer_.clear();
    oscillating_ = false;
}

bool FailureDetector::IsOscillating() const {
    return oscillating_;
}

bool FailureDetector::Detect(double v_eps, double omega_eps) {
    oscillating_ = false;

    // we start Detecting only as soon as we have the buffer filled at least half
    if (buffer_.size() < buffer_.capacity() / 2) {
        return false;
    }

    // compute mean for v and omega
    double n = (double)buffer_.size();
    double v_mean = 0;
    double omega_mean = 0;
    int omega_zero_crossings = 0;
    for (int i = 0; i < n; ++i) {
        v_mean += buffer_[i].x;
        omega_mean += buffer_[i].theta;
        if (i > 0 &&
            Sign(buffer_[i].theta) != Sign(buffer_[i - 1].theta))
            ++omega_zero_crossings;
    }
    v_mean /= n;
    omega_mean /= n;

    if (std::abs(v_mean) < v_eps && 
        std::abs(omega_mean) < omega_eps &&
        omega_zero_crossings > 1) {
        oscillating_ = true;
    }
    return oscillating_;
}

}  // namespace teb_controller
}  // namespace control
}  // namespace autonomy
