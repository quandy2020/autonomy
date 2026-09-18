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

#include "autonomy/localization/atlas/estimate/buffered_lidar_residual_source.hpp"
#include "autonomy/localization/atlas/sensor/lidar/lightning/ivox/ivox.hpp"
#include "autonomy/localization/atlas/sensor/lidar/lightning/obs_model/obs_model.hpp"
#include "autonomy/localization/atlas/sensor/types.hpp"
#include "autonomy/localization/atlas/type.hpp"

#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace autonomy::localization::atlas {
namespace sensor {

//! Lidar measurement source. Lightning algorithms live under lidar/lightning/
//! and feed residuals into the single Atlas estimator — not a second SLAM.
//! Live IVox is owned by mapping::MapIncremental; inject via set_ivox().
class LidarSensor {
public:
    struct Options {
        std::string topic = "/points";
        std::string config_path;
        bool enable_lightning_algo = false;
        bool enable_ground_prior = false;
        double ivox_resolution = 0.5;
        std::size_t max_queue = 32;
    };

    explicit LidarSensor(Options options);
    LidarSensor() : LidarSensor(Options{}) {}

    bool Start();
    void Stop();
    [[nodiscard]] bool is_running() const { return running_; }

    //! Non-owning live IVox from mapping::MapIncremental (required for lightning).
    void set_ivox(lightning::IVox* ivox) { ivox_ = ivox; }

    void Feed(const CloudSample& sample);
    //! Body points + T_wc: update IVox and push ObsModel residuals.
    void FeedWithPose(double timestamp, const Mat44_t& T_wc,
                      const std::vector<Vec3_t>& points_body);
    void FeedPoints(double timestamp, const std::vector<Vec3_t>& points_body);
    bool PopLatest(CloudSample* out);

    void PushFactors(double timestamp, estimate::LidarFactorBatch batch);

    estimate::ILidarResidualSource* residual_source() {
        return residual_source_.get();
    }
    std::shared_ptr<estimate::BufferedLidarResidualSource>
    residual_source_shared() {
        return residual_source_;
    }

    lightning::IVox* ivox() { return ivox_; }
    const Options& options() const { return options_; }

private:
    Options options_;
    bool running_ = false;
    mutable std::mutex mtx_;
    std::deque<CloudSample> queue_;
    std::shared_ptr<estimate::BufferedLidarResidualSource> residual_source_;
    lightning::IVox* ivox_ = nullptr;
    lightning::ObsModel obs_model_;
};

}  // namespace sensor
}  // namespace autonomy::localization::atlas
