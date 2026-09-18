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

//! frontend/local_estimator — realtime pose (ESKF for LO/LIO strategy).
//! Lives in Frontend; writes the same Atlas pose State (not a second SLAM).

#include "autonomy/localization/atlas/estimate/lidar_residual_source.hpp"
#include "autonomy/localization/atlas/estimate/residual_odom.hpp"
#include "autonomy/localization/atlas/sensor/lidar/lightning/eskf/eskf.hpp"
#include "autonomy/localization/atlas/type.hpp"

namespace autonomy::localization::atlas {
namespace frontend {

class LocalEstimator {
public:
    void Reset(const Mat44_t& T_wb = Mat44_t::Identity()) { eskf_.Reset(T_wb); }

    void PredictImu(double dt, const Vec3_t& gyro, const Vec3_t& acc) {
        eskf_.PredictImu(dt, gyro, acc);
    }

    //! Compose body-frame odom delta into world pose (needed for WIO / LWIO).
    void UpdateOdom(const Mat44_t& T_delta) { eskf_.UpdateOdom(T_delta); }

    int UpdateLidar(const estimate::LidarFactorBatch& batch) {
        return eskf_.UpdateLidar(batch);
    }

    [[nodiscard]] Mat44_t T_wb() const { return eskf_.T_wb(); }
    [[nodiscard]] Mat44_t T_cw() const { return eskf_.T_wb().inverse(); }

private:
    sensor::lightning::Eskf eskf_;
};

}  // namespace frontend
}  // namespace autonomy::localization::atlas
