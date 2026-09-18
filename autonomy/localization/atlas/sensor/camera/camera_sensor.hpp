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

#include <string>

namespace autonomy::localization::atlas {
namespace sensor {

//! Camera measurement source. ROS IO enters via CameraBridge; frames are
//! notified into CameraSensor for SensorSuite consumers.
class CameraSensor {
public:
    struct Options {
        std::string rgb_topic = "/camera/rgb/image_raw";
        std::string depth_topic = "/camera/depth/image_raw";
    };

    explicit CameraSensor(Options options) : options_(std::move(options)) {}
    CameraSensor() : CameraSensor(Options{}) {}

    bool Start() {
        running_ = true;
        return true;
    }
    void Stop() { running_ = false; }
    [[nodiscard]] bool is_running() const { return running_; }

    void NotifyFrame(const ImageSample& sample) { latest_ = sample; has_latest_ = true; }
    bool Latest(ImageSample* out) const {
        if (!has_latest_ || !out) {
            return false;
        }
        *out = latest_;
        return true;
    }

    const Options& options() const { return options_; }

private:
    Options options_;
    bool running_ = false;
    bool has_latest_ = false;
    ImageSample latest_{};
};

}  // namespace sensor
}  // namespace autonomy::localization::atlas
