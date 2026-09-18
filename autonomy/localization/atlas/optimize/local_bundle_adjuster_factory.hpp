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

#ifndef AUTONOMY_LOCALIZATION_ATLAS_OPTIMIZE_LOCAL_BUNDLE_ADJUSTER_FACTORY_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_OPTIMIZE_LOCAL_BUNDLE_ADJUSTER_FACTORY_HPP_

#include "autonomy/localization/atlas/optimize/local_bundle_adjuster_g2o.hpp"
#include "autonomy/localization/atlas/optimize/local_bundle_adjuster_inertial.hpp"
#include "autonomy/localization/atlas/mapping/local_joint_ba.hpp"
#include "autonomy/localization/atlas/estimate/residual_mask.hpp"
#include "autonomy/localization/atlas/sensor/imu/config.hpp"

#include <memory>
#include <stdexcept>
#include <string>

namespace autonomy::localization::atlas {

namespace optimize {

class local_bundle_adjuster_factory {
public:
    static std::unique_ptr<local_bundle_adjuster> create(
        const YAML::Node& yaml_node,
        const imu::config& imu_cfg = imu::config{},
        const estimate::ResidualMask* residual_mask = nullptr) {
        const auto& backend = yaml_node["backend"].as<std::string>("g2o");
        if (backend != "g2o" && backend != "joint") {
            throw std::runtime_error("Invalid backend: only g2o|joint supported");
        }
        if (backend == "joint" || yaml_node["use_joint"].as<bool>(false)) {
            estimate::ResidualMask mask;
            if (residual_mask) {
                mask = *residual_mask;
            } else {
                mask.vision = yaml_node["joint_vision"].as<bool>(true);
                mask.imu = yaml_node["joint_imu"].as<bool>(imu_cfg.enabled);
                mask.lidar = yaml_node["joint_lidar"].as<bool>(false);
                mask.odom = yaml_node["joint_odom"].as<bool>(false);
            }
            return std::unique_ptr<local_bundle_adjuster>(
                new mapping::LocalJointBA(yaml_node, imu_cfg, mask, nullptr));
        }
        if (imu_cfg.enabled && yaml_node["use_inertial"].as<bool>(true)) {
            return std::unique_ptr<local_bundle_adjuster>(
                new local_bundle_adjuster_inertial(yaml_node, imu_cfg));
        }
        return std::unique_ptr<local_bundle_adjuster>(new local_bundle_adjuster_g2o(yaml_node));
    }
};

} // namespace optimize
}  // namespace autonomy::localization::atlas

#endif  // AUTONOMY_LOCALIZATION_ATLAS_OPTIMIZE_LOCAL_BUNDLE_ADJUSTER_FACTORY_HPP_
