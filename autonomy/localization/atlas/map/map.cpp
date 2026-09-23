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
 * @file map.cpp
 * @brief `Map` implementation: keyframe/map-point sets and scale/gravity alignment.
 *
 * Corresponds to ORB-SLAM3 `Map.cc`.
 */

#include "autonomy/localization/atlas/map/map.hpp"

#include <algorithm>

namespace autonomy {
namespace localization {
namespace atlas {

long unsigned int Map::next_id = 0;

Map::Map() { id = next_id++; }

Map::Map(long unsigned int /*init_keyframe_id*/) { id = next_id++; }

void Map::AddKeyFrame(const std::shared_ptr<KeyFrame>& keyframe) {
    std::lock_guard<std::mutex> lock(mutex_map_);
    keyframes_.insert(keyframe);
    if (keyframe) {
        max_keyframe_id_ = std::max(max_keyframe_id_, keyframe->id);
    }
}

void Map::AddMapPoint(const std::shared_ptr<MapPoint>& map_point) {
    std::lock_guard<std::mutex> lock(mutex_map_);
    map_points_.insert(map_point);
}

void Map::EraseMapPoint(const std::shared_ptr<MapPoint>& map_point) {
    std::lock_guard<std::mutex> lock(mutex_map_);
    map_points_.erase(map_point);
}

void Map::EraseKeyFrame(const std::shared_ptr<KeyFrame>& keyframe) {
    std::lock_guard<std::mutex> lock(mutex_map_);
    keyframes_.erase(keyframe);
}

void Map::SetReferenceMapPoints(
    const std::vector<std::shared_ptr<MapPoint>>& map_points) {
    std::lock_guard<std::mutex> lock(mutex_map_);
    reference_map_points_ = map_points;
}

std::vector<std::shared_ptr<KeyFrame>> Map::GetAllKeyFrames() const {
    std::lock_guard<std::mutex> lock(mutex_map_);
    return {keyframes_.begin(), keyframes_.end()};
}

std::vector<std::shared_ptr<MapPoint>> Map::GetAllMapPoints() const {
    std::lock_guard<std::mutex> lock(mutex_map_);
    return {map_points_.begin(), map_points_.end()};
}

long unsigned int Map::MapPointsInMap() const {
    std::lock_guard<std::mutex> lock(mutex_map_);
    return map_points_.size();
}

long unsigned int Map::KeyFramesInMap() const {
    std::lock_guard<std::mutex> lock(mutex_map_);
    return keyframes_.size();
}

void Map::ApplyScaledRotation(const SE3& Twg, double scale,
                              bool update_velocity) {
    std::lock_guard<std::mutex> lock(mutex_map_);
    const Mat33 Rwg = Twg.rotation();
    for (const auto& keyframe : keyframes_) {
        if (!keyframe) {
            continue;
        }
        // Tcw_new = Tcw * [s Rwg^T | 0]  (world rotated/scaled).
        SE3 Tcw = keyframe->GetPose();
        const SE3 Twc = Tcw.inverse();
        SE3 Twc_new = SE3Identity();
        Twc_new.linear() = Rwg * Twc.rotation();
        Twc_new.translation() = scale * Rwg * Twc.translation();
        keyframe->SetPose(Twc_new.inverse());
        if (update_velocity && keyframe->has_velocity) {
            keyframe->velocity_world = scale * Rwg * keyframe->velocity_world;
        }
    }
    for (const auto& map_point : map_points_) {
        if (!map_point || map_point->isBad()) {
            continue;
        }
        map_point->SetWorldPos(scale * Rwg * map_point->GetWorldPos());
        map_point->UpdateNormalAndDepth();
    }
}

void Map::IncreaseChangeIndex() {
    std::lock_guard<std::mutex> lock(mutex_map_);
    ++big_change_index_;
}

int Map::GetLastBigChangeIdx() const {
    std::lock_guard<std::mutex> lock(mutex_map_);
    return big_change_index_;
}

void Map::clear() {
    std::lock_guard<std::mutex> lock(mutex_map_);
    keyframes_.clear();
    map_points_.clear();
    reference_map_points_.clear();
    keyframe_origins.clear();
    max_keyframe_id_ = 0;
    imu_initialized_ = false;
    inertial_ba1_ = false;
    inertial_ba2_ = false;
    big_change_index_ = 0;
}

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
