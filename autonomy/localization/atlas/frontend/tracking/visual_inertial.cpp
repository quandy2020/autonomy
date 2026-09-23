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

/**
 * @file visual_inertial.cpp
 * @brief VisualInertial implementation: delegates to tracking::Tracker in IMU+RGB-D mode.
 */

#include "autonomy/localization/atlas/frontend/tracking/visual_inertial.hpp"

namespace autonomy {
namespace localization {
namespace atlas {

bool VisualInertial::Init(const AtlasConfig& config) {
    tracking::Tracker::Options options;
    options.sensor = tracking::Tracker::SensorFromConfig(config, true);
    return tracker_.Init(config, options);
}

bool VisualInertial::Process(const SensorData& data) {
    return tracker_.Process(data);
}

bool VisualInertial::GetResult(OdometryResult* out) const {
    return tracker_.GetResult(out);
}

void VisualInertial::Reset() { tracker_.Reset(); }

bool VisualInertial::ConsumePendingKeyframe(Keyframe* out) {
    return tracker_.ConsumePendingKeyframe(out);
}

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
