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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_OPTIMIZE_LOCAL_BUNDLE_ADJUSTER_INERTIAL_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_OPTIMIZE_LOCAL_BUNDLE_ADJUSTER_INERTIAL_HPP_

#include "autonomy/localization/atlas/sensor/imu/config.hpp"
#include "autonomy/localization/atlas/optimize/local_bundle_adjuster.hpp"
#include "autonomy/localization/atlas/optimize/local_bundle_adjuster_g2o.hpp"

#include <memory>

#include "yaml-cpp/yaml.h"

namespace autonomy::localization::atlas {
namespace optimize {

//! Local BA with IMU preintegration factors. Falls back to visual-only g2o BA until
//! inertial initialization has succeeded and keyframes carry valid preintegrators.
class local_bundle_adjuster_inertial final : public local_bundle_adjuster {
public:
    local_bundle_adjuster_inertial(const YAML::Node& yaml_node, const imu::config& imu_cfg);

    void optimize(data::map_database* map_db,
                  const std::shared_ptr<data::keyframe>& curr_keyfrm,
                  bool* const force_stop_flag) const override;

    void set_inertial_ready(bool ready) { inertial_ready_ = ready; }
    bool inertial_ready() const { return inertial_ready_; }
    void set_gravity(const Vec3_t& g);

private:
    void optimize_inertial(data::map_database* map_db,
                           const std::shared_ptr<data::keyframe>& curr_keyfrm,
                           bool* const force_stop_flag) const;

    local_bundle_adjuster_g2o visual_ba_;
    imu::config imu_cfg_;
    mutable bool inertial_ready_ = false;
    Vec3_t gravity_ = Vec3_t(0.0, 0.0, -9.81);
    unsigned int num_first_iter_ = 5;
    unsigned int num_second_iter_ = 10;
};

}  // namespace optimize
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_OPTIMIZE_LOCAL_BUNDLE_ADJUSTER_INERTIAL_HPP_
