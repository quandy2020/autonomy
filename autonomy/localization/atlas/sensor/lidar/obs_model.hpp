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

//! sensor/lidar/obs_model — point-plane (+ optional P2P ICP) residuals.

#include "autonomy/localization/atlas/estimate/lidar_residual_source.hpp"
#include "autonomy/localization/atlas/mapping/ivox/ivox.hpp"
#include "autonomy/localization/atlas/type.hpp"

#include <vector>

namespace autonomy::localization::atlas {
namespace sensor {

/**
 * Point-plane observation model (measurement only — not a second SLAM).
 * Builds residual material for LocalEstimator IEKF / Joint BA.
 */
class ObsModel {
public:
    struct Options {
        double max_distance = 0.5;
        int max_residuals = 2000;
        bool enable_ground_prior = false;
        double ground_weight = 0.05;
        //! lightning enable_icp_part: add point-to-point to IEKF.
        bool enable_icp_part = true;
        double plane_weight = 1.0;
        double icp_weight = 0.1;
        //! Max |e| for P2P (m); lightning rejects > 0.5.
        double icp_max_distance = 0.5;
        //! lightning: keep surf if ||p_body||^2 > 81 * pd2^2.
        double srange_scale = 81.0;
        //! Lidar→IMU extrinsic (p_imu = R p_lidar + t). Applied in
        //! BuildAgainstIVox like lightning offset_R/t.
        Mat44_t T_imu_lidar = Mat44_t::Identity();
    };

    ObsModel() = default;
    explicit ObsModel(Options options) : options_(std::move(options)) {}

    void set_options(Options options) { options_ = std::move(options); }
    [[nodiscard]] const Options& options() const { return options_; }

    estimate::LidarFactorBatch Build(
        const std::vector<Vec3_t>& points_body,
        const std::vector<Vec3_t>& normals_world,
        const std::vector<double>& plane_d) const;

    //! Correspond body points to IVox planes given T_wc (body→world).
    estimate::LidarFactorBatch BuildAgainstIVox(
        const Mat44_t& T_wc,
        const std::vector<Vec3_t>& points_body,
        const mapping::IVox& map) const;

    //! Ground prior: PCA plane from lowest 20% z (fallback Z=0).
    estimate::LidarFactorBatch BuildStub(
        const std::vector<Vec3_t>& points_body) const;

private:
    Options options_;
};

}  // namespace sensor
}  // namespace autonomy::localization::atlas
