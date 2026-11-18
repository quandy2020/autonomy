/*
 * Copyright 2026 The Openbot Authors
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
 * @file pose_filter.hpp
 * @brief First-order pose filter (port of opennav_docking::PoseFilter).
 */

#ifndef AUTONOMY_TASK_CHARGING_DOCK_POSE_FILTER_HPP_
#define AUTONOMY_TASK_CHARGING_DOCK_POSE_FILTER_HPP_

#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>

namespace autonomy {
namespace task {
namespace charging {

/**
 * @class autonomy::task::charging::PoseFilter
 * @brief Lerp position + slerp orientation; reset when the stamp gap exceeds
 *        timeout or the frame changes.
 */
class PoseFilter {
public:
    /**
     * @brief Construct a filter.
     * @param[in] coef Blend weight toward the measurement in (0, 1]; <=0 disables.
     * @param[in] timeout_sec Reset if successive stamps differ by more than this.
     */
    PoseFilter(double coef, double timeout_sec);

    /**
     * @brief Incorporate one measurement.
     * @param[in] measurement Detected pose with header stamp/frame.
     * @return Filtered pose.
     */
    ::automsgs::msgs::geometry_msgs::PoseStamped Update(
        const ::automsgs::msgs::geometry_msgs::PoseStamped& measurement);

private:
    double coef_{0.1};
    double timeout_sec_{1.0};
    bool valid_{false};
    ::automsgs::msgs::geometry_msgs::PoseStamped pose_;
};

}  // namespace charging
}  // namespace task
}  // namespace autonomy

#endif  // AUTONOMY_TASK_CHARGING_DOCK_POSE_FILTER_HPP_
