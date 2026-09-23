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
 * @file multi_map.cpp
 * @brief `MultiMap` implementation: create/switch maps and prune bad maps.
 *
 * Plays the role of ORB-SLAM3 `Atlas.cc`.
 */

#include "autonomy/localization/atlas/map/multi_map.hpp"

namespace autonomy {
namespace localization {
namespace atlas {

MultiMap::MultiMap() { CreateNewMap(); }

MultiMap::MultiMap(long unsigned int init_keyframe_id)
    : last_init_keyframe_id_(init_keyframe_id) {
    CreateNewMap();
}

void MultiMap::CreateNewMap() {
    std::lock_guard<std::mutex> lock(mutex_);
    current_map_ = std::make_shared<Map>(last_init_keyframe_id_);
    maps_.insert(current_map_);
}

void MultiMap::ChangeMap(const std::shared_ptr<Map>& map) {
    if (!map) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    maps_.insert(map);
    current_map_ = map;
}

void MultiMap::AddKeyFrame(const std::shared_ptr<KeyFrame>& keyframe) {
    auto map = GetCurrentMap();
    if (map) {
        map->AddKeyFrame(keyframe);
    }
}

void MultiMap::AddMapPoint(const std::shared_ptr<MapPoint>& map_point) {
    auto map = GetCurrentMap();
    if (map) {
        map->AddMapPoint(map_point);
    }
}

void MultiMap::SetReferenceMapPoints(
    const std::vector<std::shared_ptr<MapPoint>>& map_points) {
    auto map = GetCurrentMap();
    if (map) {
        map->SetReferenceMapPoints(map_points);
    }
}

std::shared_ptr<Map> MultiMap::GetCurrentMap() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!current_map_) {
        current_map_ = std::make_shared<Map>(last_init_keyframe_id_);
        maps_.insert(current_map_);
    }
    return current_map_;
}

Map* MultiMap::mutable_current_map() { return GetCurrentMap().get(); }

std::vector<std::shared_ptr<Map>> MultiMap::GetAllMaps() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return std::vector<std::shared_ptr<Map>>(maps_.begin(), maps_.end());
}

int MultiMap::CountMaps() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return static_cast<int>(maps_.size());
}

std::vector<std::shared_ptr<KeyFrame>> MultiMap::GetAllKeyFrames() const {
    auto map = const_cast<MultiMap*>(this)->GetCurrentMap();
    return map ? map->GetAllKeyFrames()
               : std::vector<std::shared_ptr<KeyFrame>>{};
}

std::vector<std::shared_ptr<MapPoint>> MultiMap::GetAllMapPoints() const {
    auto map = const_cast<MultiMap*>(this)->GetCurrentMap();
    return map ? map->GetAllMapPoints()
               : std::vector<std::shared_ptr<MapPoint>>{};
}

long unsigned int MultiMap::MapPointsInMap() const {
    auto map = const_cast<MultiMap*>(this)->GetCurrentMap();
    return map ? map->MapPointsInMap() : 0;
}

long unsigned int MultiMap::KeyFramesInMap() const {
    auto map = const_cast<MultiMap*>(this)->GetCurrentMap();
    return map ? map->KeyFramesInMap() : 0;
}

void MultiMap::SetMapBad(const std::shared_ptr<Map>& map) {
    if (!map) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    maps_.erase(map);
    bad_maps_.insert(map);
    if (current_map_ == map) {
        current_map_.reset();
    }
}

void MultiMap::RemoveBadMaps() {
    std::lock_guard<std::mutex> lock(mutex_);
    bad_maps_.clear();
}

void MultiMap::clearMap() {
    auto map = GetCurrentMap();
    if (map) {
        map->clear();
    }
}

void MultiMap::clearAtlas() {
    std::lock_guard<std::mutex> lock(mutex_);
    maps_.clear();
    bad_maps_.clear();
    current_map_.reset();
}

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
