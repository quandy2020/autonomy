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

//! frontend/lio/imu_process — IMU predict + constant-ω deskew skeleton (MVP).

#include "autonomy/localization/atlas/frontend/local_estimator.hpp"
#include "autonomy/localization/atlas/sensor/types.hpp"
#include "autonomy/localization/atlas/type.hpp"

#include <cmath>
#include <deque>
#include <vector>

namespace autonomy::localization::atlas {
namespace frontend {
namespace lio {

class ImuProcess {
public:
    void SetExtrinsic(const Mat44_t& T_il = Mat44_t::Identity()) {
        T_imu_lidar_ = T_il;
    }

    [[nodiscard]] const Mat44_t& T_imu_lidar() const { return T_imu_lidar_; }

    //! Propagate LocalEstimator with successive IMU samples (uses sample Δt).
    void ProcessPredict(LocalEstimator* estimator,
                        const std::deque<sensor::ImuSample>& imu) const {
        if (!estimator || imu.size() < 2) {
            return;
        }
        for (std::size_t i = 1; i < imu.size(); ++i) {
            const double dt = imu[i].timestamp - imu[i - 1].timestamp;
            if (dt <= 0.0 || dt > 0.5) {
                continue;
            }
            estimator->PredictImu(dt, imu[i].gyro, imu[i].acc);
        }
    }

    //! Given imu queue + fixed dt, call estimator PredictImu once per sample.
    void ProcessPredict(LocalEstimator* estimator,
                        const std::deque<sensor::ImuSample>& imu,
                        double dt) const {
        if (!estimator || imu.empty() || dt <= 0.0) {
            return;
        }
        for (const auto& s : imu) {
            estimator->PredictImu(dt, s.gyro, s.acc);
        }
    }

    //! Constant-ω deskew: for each point p at ratio α, R(α)*p.
    //! If point_time_rel empty / size mismatch → no-op copy.
    [[nodiscard]] std::vector<Vec3_t> UndistortScan(
        const std::vector<Vec3_t>& points_body,
        const std::vector<double>& point_time_rel,
        const Mat44_t& /*T_begin*/,
        const Vec3_t& omega) const {
        if (points_body.empty() || point_time_rel.size() != points_body.size()) {
            return points_body;
        }
        const double wnorm = omega.norm();
        if (wnorm < 1e-12) {
            return points_body;
        }
        const Vec3_t axis = omega / wnorm;
        std::vector<Vec3_t> out;
        out.reserve(points_body.size());
        (void)T_imu_lidar_;
        for (std::size_t i = 0; i < points_body.size(); ++i) {
            double alpha = point_time_rel[i];
            if (!std::isfinite(alpha)) {
                out.push_back(points_body[i]);
                continue;
            }
            if (alpha < 0.0) {
                alpha = 0.0;
            } else if (alpha > 1.0) {
                alpha = 1.0;
            }
            const double angle = wnorm * alpha;
            const Mat33_t R_a =
                Eigen::AngleAxisd(angle, axis).toRotationMatrix();
            out.push_back(R_a * points_body[i]);
        }
        return out;
    }

private:
    //! Lidar←IMU extrinsic (stored for future body-frame transforms).
    Mat44_t T_imu_lidar_ = Mat44_t::Identity();
};

}  // namespace lio
}  // namespace frontend
}  // namespace autonomy::localization::atlas
