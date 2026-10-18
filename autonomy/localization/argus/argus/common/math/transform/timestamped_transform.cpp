/*
 * Copyright 2026 The Openbot Authors
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

#include "argus/common/math/transform/timestamped_transform.hpp"

#include <cassert>

#include <Eigen/Geometry>

namespace argus {
namespace transform {

TimestampedTransform Interpolate(const TimestampedTransform& start,
                                 const TimestampedTransform& end,
                                 const double time_sec) {
    assert(start.time_sec <= time_sec);
    assert(end.time_sec >= time_sec);

    const double duration = end.time_sec - start.time_sec;
    const double factor =
        (duration <= 0.0) ? 0.0 : (time_sec - start.time_sec) / duration;
    const Eigen::Vector3d origin =
        start.transform.translation() +
        (end.transform.translation() - start.transform.translation()) * factor;
    const Eigen::Quaterniond rotation =
        Eigen::Quaterniond(start.transform.rotation())
            .slerp(factor, Eigen::Quaterniond(end.transform.rotation()));
    return TimestampedTransform{time_sec, Rigid3d(origin, rotation)};
}

}  // namespace transform
}  // namespace argus
