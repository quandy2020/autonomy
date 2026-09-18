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

#include "autonomy/localization/atlas/estimate/residual_odom.hpp"
#include "autonomy/localization/atlas/sensor/types.hpp"

#include <deque>
#include <memory>
#include <mutex>
#include <string>

namespace autonomy::localization::atlas {
namespace sensor {

//! Wheel / external odometry measurement source (not a second pose authority).
class OdomSensor {
public:
    struct Options {
        std::string topic = "/wheel_odom";
        std::size_t max_queue = 200;
        bool push_residuals = true;
    };

    explicit OdomSensor(Options options);
    OdomSensor() : OdomSensor(Options{}) {}

    bool Start();
    void Stop();
    [[nodiscard]] bool is_running() const { return running_; }

    void Feed(const OdomSample& sample);
    bool PopLatest(OdomSample* out);

    estimate::IOdomResidualSource* residual_source() {
        return residual_source_.get();
    }

    const Options& options() const { return options_; }

private:
    Options options_;
    bool running_ = false;
    mutable std::mutex mtx_;
    std::deque<OdomSample> queue_;
    std::shared_ptr<estimate::BufferedOdomResidualSource> residual_source_;
    bool has_last_ = false;
    Mat44_t last_T_ = Mat44_t::Identity();
};

}  // namespace sensor
}  // namespace autonomy::localization::atlas
