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

#pragma once

#include <boost/circular_buffer.hpp>

#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/macros.hpp"

namespace autonomy {
namespace control {
namespace teb_controller {

/**
 * @brief Detect stuck or oscillating motion from velocity history
 */
class FailureDetector
{
public:
    /**
     * Define FailureDetector::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(FailureDetector);

    /**
     * @brief Set velocity history buffer length
     * @param length Number of measurements to keep
     */
    void SetBufferLength(int length) {
        buffer_.set_capacity(length);
    }

    /**
     * @brief Add velocity sample and update detection state
     * @param twist Current velocity command
     * @param v_max Maximum forward velocity
     * @param v_backwards_max Maximum backward velocity
     * @param omega_max Maximum angular velocity
     * @param v_eps Normalized linear velocity threshold
     * @param omega_eps Normalized angular velocity threshold
     */
    void Update(const autonomy::commsgs::geometry_msgs::Twist& twist,
                double v_max, double v_backwards_max, double omega_max,
                double v_eps, double omega_eps);

    /**
     * @brief Check if oscillation was detected
     * @return true if oscillating, false otherwise
     */
    bool IsOscillating() const;

    /**
     * @brief Reset internal state and buffer
     */
    void Clear();

protected:
    /**
     * @brief Run stuck/oscillation detection on the buffer
     * @param v_eps Normalized linear velocity threshold
     * @param omega_eps Normalized angular velocity threshold
     * @return true if stuck or oscillating
     */
    bool Detect(double v_eps, double omega_eps);

private:
    // Recent velocity samples
    boost::circular_buffer<commsgs::geometry_msgs::Twist2D> buffer_;

    // True when oscillation is detected
    bool oscillating_ = false;
};

}  // namespace teb_controller
}  // namespace control
}  // namespace autonomy
