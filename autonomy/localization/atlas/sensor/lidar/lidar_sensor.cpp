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

#include "autonomy/localization/atlas/sensor/lidar/lidar_sensor.hpp"

#include "glog/logging.h"

namespace autonomy::localization::atlas {
namespace sensor {

LidarSensor::LidarSensor(Options options)
    : options_(std::move(options)),
      residual_source_(
          std::make_shared<estimate::BufferedLidarResidualSource>()) {
    lightning::ObsModel::Options obs_opts;
    obs_opts.max_distance = 0.5;
    obs_opts.max_residuals = 2000;
    obs_opts.enable_ground_prior = options_.enable_ground_prior;
    obs_opts.ground_weight = 0.05;
    obs_model_ = lightning::ObsModel(obs_opts);
}

bool LidarSensor::Start() {
    running_ = true;
    if (options_.enable_lightning_algo) {
        LOG(INFO) << "LidarSensor: lightning ObsModel enabled, topic="
                  << options_.topic
                  << " res=" << options_.ivox_resolution
                  << " (IVox via MapIncremental::set_ivox)";
    } else {
        LOG(INFO) << "LidarSensor: started (measurement + residual buffer), topic="
                  << options_.topic;
    }
    return true;
}

void LidarSensor::Stop() {
    running_ = false;
    std::lock_guard<std::mutex> lock(mtx_);
    queue_.clear();
    if (ivox_) {
        ivox_->Clear();
    }
}

void LidarSensor::Feed(const CloudSample& sample) {
    std::lock_guard<std::mutex> lock(mtx_);
    queue_.push_back(sample);
    while (queue_.size() > options_.max_queue) {
        queue_.pop_front();
    }
}

void LidarSensor::FeedWithPose(double timestamp, const Mat44_t& T_wc,
                               const std::vector<Vec3_t>& points_body) {
    CloudSample sample;
    sample.timestamp = timestamp;
    sample.points_body = points_body;
    Feed(sample);

    if (!options_.enable_lightning_algo || points_body.empty() || !ivox_) {
        return;
    }

    const Mat33_t R_wc = T_wc.block<3, 3>(0, 0);
    const Vec3_t t_wc = T_wc.block<3, 1>(0, 3);
    std::vector<Vec3_t> points_world;
    points_world.reserve(points_body.size());
    for (const auto& p : points_body) {
        if (p.allFinite()) {
            points_world.push_back(R_wc * p + t_wc);
        }
    }
    // Build residuals against *existing* map, then insert current scan.
    auto batch = obs_model_.BuildAgainstIVox(T_wc, points_body, *ivox_);
    ivox_->InsertWorldPoints(points_world);
    if (batch.empty() && options_.enable_ground_prior) {
        batch = obs_model_.BuildStub(points_body);
    }
    if (!batch.empty()) {
        residual_source_->Push(timestamp, std::move(batch));
    }
}

void LidarSensor::FeedPoints(double timestamp,
                             const std::vector<Vec3_t>& points_body) {
    // Without pose: identity T_wc (sensor-frame map growth for unit tests).
    FeedWithPose(timestamp, Mat44_t::Identity(), points_body);
}

bool LidarSensor::PopLatest(CloudSample* out) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!out || queue_.empty()) {
        return false;
    }
    *out = queue_.back();
    return true;
}

void LidarSensor::PushFactors(double timestamp,
                              estimate::LidarFactorBatch batch) {
    if (!batch.empty()) {
        residual_source_->Push(timestamp, std::move(batch));
    }
}

}  // namespace sensor
}  // namespace autonomy::localization::atlas
