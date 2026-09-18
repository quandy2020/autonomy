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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_INITIALIZE_INERTIAL_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_INITIALIZE_INERTIAL_HPP_

#include "autonomy/localization/atlas/sensor/imu/bias.hpp"
#include "autonomy/localization/atlas/sensor/imu/config.hpp"
#include "autonomy/localization/atlas/sensor/imu/preintegrator.hpp"
#include "autonomy/localization/atlas/type.hpp"

#include <memory>
#include <vector>

namespace autonomy::localization::atlas {

namespace data {
class keyframe;
class map_database;
}  // namespace data

namespace initialize {

//! Visual-inertial MAP initialization (ORB-SLAM3-style):
//! estimate scale, gravity, velocities and biases, then write back to the map.
class inertial {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum class state_t {
        NotReady,
        Estimating,
        Succeeded,
        Failed
    };

    explicit inertial(const imu::config& cfg);

    void reset();
    state_t state() const { return state_; }

    void add_keyframe(const std::shared_ptr<data::keyframe>& keyfrm);

    //! Closed-form inertial solve once enough keyframes are available.
    bool try_initialize();

    //! Apply scale / gravity / velocities / biases to map databases.
    bool apply_to_map(data::map_database* map_db);

    double scale() const { return scale_; }
    Vec3_t gravity() const { return gravity_; }
    imu::bias bias() const { return bias_; }
    const eigen_alloc_vector<Vec3_t>& velocities() const { return velocities_; }
    bool succeeded() const { return state_ == state_t::Succeeded; }

    unsigned int min_keyframes() const { return min_keyframes_; }
    void set_min_keyframes(unsigned int n) { min_keyframes_ = n; }

private:
    bool estimate_inertial_parameters();
    bool refine_with_preintegration_ba();

    imu::config cfg_;
    state_t state_ = state_t::NotReady;
    unsigned int min_keyframes_ = 10;

    std::vector<std::shared_ptr<data::keyframe>> keyfrms_;
    eigen_alloc_vector<Vec3_t> velocities_;

    double scale_ = 1.0;
    Vec3_t gravity_ = Vec3_t(0.0, 0.0, -9.81);
    imu::bias bias_{};
};

}  // namespace initialize
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_INITIALIZE_INERTIAL_HPP_
