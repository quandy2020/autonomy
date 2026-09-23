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
 * @file map_manager.cpp
 * @brief `MapManager` implementation: lightweight Keyframe / Landmark list maintenance.
 *
 * Not an ORB-SLAM3 graph map; intended for a simplified frontend handoff.
 */

#include "autonomy/localization/atlas/map/map_manager.hpp"

#include <algorithm>

namespace autonomy {
namespace localization {
namespace atlas {

bool MapManager::Init(const AtlasConfig& config) {
    config_ = config;
    Reset();
    return true;
}

void MapManager::Reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    keyframes_.clear();
    landmarks_.clear();
    next_keyframe_id_ = 0;
}

void MapManager::AddKeyframe(const Keyframe& keyframe) {
    std::lock_guard<std::mutex> lock(mutex_);
    Keyframe stored = keyframe;
    if (stored.id < 0) {
        stored.id = next_keyframe_id_++;
    } else {
        next_keyframe_id_ = std::max(next_keyframe_id_, stored.id + 1);
    }
    for (const auto& landmark : stored.landmarks) {
        landmarks_.push_back(landmark);
    }
    keyframes_.push_back(std::move(stored));
}

void MapManager::UpdateFromOdometry(const OdometryResult& odometry) {
    if (!odometry.valid) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& landmark : odometry.landmarks) {
        landmarks_.push_back(landmark);
    }
}

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
