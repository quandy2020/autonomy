/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
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

#ifndef AUTONOMY_AUDIO_INFERENCE_DIRECTION_DETECTION_HPP_
#define AUTONOMY_AUDIO_INFERENCE_DIRECTION_DETECTION_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/Dense"
// Eigen 3.3.x may define ALIVE; clashes with some DDS headers.
#if defined(ALIVE)
#undef ALIVE
#endif

#include "automsgs/msgs/geometry_msgs/point.pb.h"

namespace autonomy {
namespace audio {

class DirectionDetection {
public:
    DirectionDetection() = default;
    ~DirectionDetection() = default;

    // Estimates sound-source position (vehicle frame) and azimuth (radians).
    std::pair<automsgs::msgs::geometry_msgs::Point, double> EstimateSoundSource(
        std::vector<std::vector<double>>&& channels_vec,
        const std::string& respeaker_extrinsic_file, int sample_rate,
        double mic_distance);

private:
    static constexpr double kSoundSpeed = 343.2;
    static constexpr int kDistance = 50;

    double EstimateDirection(std::vector<std::vector<double>>&& channels_vec,
                             int sample_rate, double mic_distance);

    bool LoadExtrinsics(const std::string& yaml_file,
                        Eigen::Matrix4d* respeaker_extrinsic);

    double GccPhat(const std::vector<double>& sig,
                   const std::vector<double>& refsig, int fs, double max_tau,
                   int interp);

    std::unique_ptr<Eigen::Matrix4d> respeaker2imu_ptr_;
};

}  // namespace audio
}  // namespace autonomy

#endif  // AUTONOMY_AUDIO_INFERENCE_DIRECTION_DETECTION_HPP_
