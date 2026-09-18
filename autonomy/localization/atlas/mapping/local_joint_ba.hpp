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

#pragma once

//! mapping/local_joint_ba — config-driven Local Joint BA (§2b).

#include "autonomy/localization/atlas/estimate/lidar_residual_source.hpp"
#include "autonomy/localization/atlas/estimate/residual_lidar.hpp"
#include "autonomy/localization/atlas/estimate/residual_mask.hpp"
#include "autonomy/localization/atlas/estimate/residual_odom.hpp"
#include "autonomy/localization/atlas/sensor/imu/config.hpp"
#include "autonomy/localization/atlas/optimize/local_bundle_adjuster.hpp"
#include "autonomy/localization/atlas/optimize/local_bundle_adjuster_g2o.hpp"
#include "autonomy/localization/atlas/optimize/local_bundle_adjuster_inertial.hpp"

#include <memory>

#include "autolink/common/log.hpp"
#include "yaml-cpp/yaml.h"

namespace autonomy::localization::atlas {
namespace mapping {

/**
 * Local Joint BA for the single AtlasSystem (LIVO joint problem).
 *
 * Same ResidualMask and State as GlobalJointBA. One optimize() may run
 * multi-stage: vision/imu (inner g2o / inertial) then lidar/odom refine.
 * That staged refine IS Atlas joint BA — not a second SLAM. Pure LO/LIO
 * without vision uses frontend::LocalEstimator (ESKF) instead.
 */
class LocalJointBA final : public optimize::local_bundle_adjuster {
public:
    LocalJointBA(const YAML::Node& yaml_node,
                 const imu::config& imu_cfg,
                 estimate::ResidualMask mask,
                 estimate::ILidarResidualSource* lidar_src,
                 estimate::IOdomResidualSource* odom_src = nullptr)
        : mask_(mask), lidar_src_(lidar_src), odom_src_(odom_src) {
        num_lidar_iter_ = yaml_node["joint_lidar_iter"].as<int>(5);
        if (imu_cfg.enabled && yaml_node["use_inertial"].as<bool>(true) &&
            mask_.imu) {
            inner_ = std::make_unique<optimize::local_bundle_adjuster_inertial>(
                yaml_node, imu_cfg);
        } else {
            inner_ = std::make_unique<optimize::local_bundle_adjuster_g2o>(
                yaml_node);
        }
    }

    void set_residual_mask(estimate::ResidualMask mask) { mask_ = mask; }
    [[nodiscard]] estimate::ResidualMask residual_mask() const { return mask_; }

    void set_lidar_residual_source(estimate::ILidarResidualSource* src) {
        lidar_src_ = src;
    }
    void set_odom_residual_source(estimate::IOdomResidualSource* src) {
        odom_src_ = src;
    }

    void optimize(data::map_database* map_db,
                  const std::shared_ptr<data::keyframe>& curr_keyfrm,
                  bool* const force_stop_flag) const override {
        if (inner_ && (mask_.vision || mask_.imu)) {
            inner_->optimize(map_db, curr_keyfrm, force_stop_flag);
        }
        if (!curr_keyfrm) {
            return;
        }
        const double t = curr_keyfrm->timestamp_;
        if (mask_.lidar && lidar_src_) {
            const auto batch = lidar_src_->Pull(t - 0.5, t + 0.05);
            if (!batch.empty()) {
                const int n = estimate::ApplyLidarPointPlaneRefine(
                    curr_keyfrm.get(), batch, num_lidar_iter_);
                AINFO << "LocalJointBA: lidar edges=" << n;
            }
        }
        if (mask_.odom && odom_src_) {
            const auto batch = odom_src_->Pull(t - 0.5, t + 0.05);
            if (!batch.empty()) {
                estimate::ApplyOdomDeltaRefine(curr_keyfrm.get(), batch);
            }
        }
    }

    void set_inertial_ready(bool ready) {
        if (auto* inertial =
                dynamic_cast<optimize::local_bundle_adjuster_inertial*>(
                    inner_.get())) {
            inertial->set_inertial_ready(ready);
        }
    }

    void set_gravity(const Vec3_t& g) {
        if (auto* inertial =
                dynamic_cast<optimize::local_bundle_adjuster_inertial*>(
                    inner_.get())) {
            inertial->set_gravity(g);
        }
    }

private:
    estimate::ResidualMask mask_{};
    estimate::ILidarResidualSource* lidar_src_ = nullptr;
    estimate::IOdomResidualSource* odom_src_ = nullptr;
    int num_lidar_iter_ = 5;
    std::unique_ptr<optimize::local_bundle_adjuster> inner_;
};

}  // namespace mapping
}  // namespace autonomy::localization::atlas
