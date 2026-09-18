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

#include "autonomy/localization/atlas/sensor/types.hpp"

#include <deque>
#include <mutex>
#include <string>

namespace autonomy::localization::atlas {
namespace sensor {

//! IMU measurement source (buffer only — not a second estimator).
class ImuSensor {
public:
    struct Options {
        std::string topic = "/imu";
        std::size_t max_queue = 2000;
    };

    explicit ImuSensor(Options options);
    ImuSensor() : ImuSensor(Options{}) {}

    bool Start();
    void Stop();
    [[nodiscard]] bool is_running() const { return running_; }

    void Feed(const ImuSample& sample);
    bool PopLatest(ImuSample* out);
    void Clear();

    const Options& options() const { return options_; }

private:
    Options options_;
    bool running_ = false;
    mutable std::mutex mtx_;
    std::deque<ImuSample> queue_;
};

}  // namespace sensor
}  // namespace autonomy::localization::atlas
