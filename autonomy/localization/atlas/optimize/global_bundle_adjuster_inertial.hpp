/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_OPTIMIZE_GLOBAL_BUNDLE_ADJUSTER_INERTIAL_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_OPTIMIZE_GLOBAL_BUNDLE_ADJUSTER_INERTIAL_HPP_

#include "autonomy/localization/atlas/sensor/imu/config.hpp"
#include "autonomy/localization/atlas/type.hpp"

#include <memory>
#include <vector>

namespace autonomy::localization::atlas {

namespace data {
class keyframe;
class map_database;
}  // namespace data

namespace optimize {

//! Full-map inertial refine after loop closure / global visual BA.
class global_bundle_adjuster_inertial {
public:
    explicit global_bundle_adjuster_inertial(const imu::config& imu_cfg,
                                             unsigned int num_iter = 10);

    void set_gravity(const Vec3_t& g) { gravity_ = g; }

    //! Optimize poses/velocities/biases of all keyframes that have temporal IMU links.
    bool optimize(data::map_database* map_db, bool* force_stop_flag = nullptr) const;

private:
    imu::config imu_cfg_;
    unsigned int num_iter_ = 10;
    Vec3_t gravity_ = Vec3_t(0.0, 0.0, -9.81);
};

}  // namespace optimize
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_OPTIMIZE_GLOBAL_BUNDLE_ADJUSTER_INERTIAL_HPP_
