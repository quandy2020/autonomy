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
 * @file map_point.cpp
 * @brief `MapPoint` implementation: observations, descriptors, normal/depth, fuse/replace.
 *
 * Corresponds to ORB-SLAM3 `MapPoint.cc`.
 */

#include "autonomy/localization/atlas/map/map_point.hpp"

#include <algorithm>
#include <cmath>
#include <climits>
#include <vector>

#include "autonomy/localization/atlas/frontend/match/orb_matcher.hpp"
#include "autonomy/localization/atlas/map/keyframe.hpp"
#include "autonomy/localization/atlas/map/map.hpp"

namespace autonomy {
namespace localization {
namespace atlas {

long unsigned int MapPoint::next_id = 0;

MapPoint::MapPoint(const Vec3& position_world,
                   const std::shared_ptr<KeyFrame>& ref, Map* map)
    : position_world_(position_world),
      reference_keyframe_(ref),
      map_(map) {
    id = next_id++;
    if (ref) {
        first_keyframe_id = ref->id;
        normal_ = (position_world_ - ref->GetCameraCenter()).normalized();
    }
}

std::shared_ptr<KeyFrame> MapPoint::GetReferenceKeyFrame() const {
    return reference_keyframe_.lock();
}

void MapPoint::SetWorldPos(const Vec3& position_world) {
    std::lock_guard<std::mutex> lock(mutex_position_);
    position_world_ = position_world;
}

Vec3 MapPoint::GetWorldPos() const {
    std::lock_guard<std::mutex> lock(mutex_position_);
    return position_world_;
}

Vec3 MapPoint::GetNormal() const {
    std::lock_guard<std::mutex> lock(mutex_position_);
    return normal_;
}

void MapPoint::AddObservation(const std::shared_ptr<KeyFrame>& keyframe,
                              int index) {
    if (!keyframe) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_features_);
    observations_[keyframe] = index;
    observation_count_ = static_cast<int>(observations_.size());
}

void MapPoint::EraseObservation(const std::shared_ptr<KeyFrame>& keyframe) {
    if (!keyframe) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_features_);
    observations_.erase(keyframe);
    observation_count_ = static_cast<int>(observations_.size());
    if (observation_count_ == 0) {
        bad_ = true;
    }
}

std::map<std::weak_ptr<KeyFrame>, int,
         std::owner_less<std::weak_ptr<KeyFrame>>>
MapPoint::GetObservations() const {
    std::lock_guard<std::mutex> lock(mutex_features_);
    return observations_;
}

int MapPoint::Observations() const {
    std::lock_guard<std::mutex> lock(mutex_features_);
    return observation_count_;
}

bool MapPoint::IsInKeyFrame(const std::shared_ptr<KeyFrame>& keyframe) const {
    if (!keyframe) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_features_);
    return observations_.count(keyframe) > 0;
}

void MapPoint::Replace(const std::shared_ptr<MapPoint>& map_point) {
    if (!map_point || map_point.get() == this) {
        return;
    }
    std::map<std::weak_ptr<KeyFrame>, int,
             std::owner_less<std::weak_ptr<KeyFrame>>>
        observations;
    {
        std::lock_guard<std::mutex> lock(mutex_features_);
        observations = observations_;
        bad_ = true;
        replaced_ = map_point;
        observations_.clear();
        observation_count_ = 0;
    }
    for (const auto& [weak_kf, index] : observations) {
        auto keyframe = weak_kf.lock();
        if (!keyframe) {
            continue;
        }
        if (!map_point->IsInKeyFrame(keyframe)) {
            keyframe->ReplaceMapPointMatch(index, map_point);
            map_point->AddObservation(keyframe, index);
        } else {
            keyframe->EraseMapPointMatch(index);
        }
    }
    map_point->ComputeDistinctiveDescriptors();
    if (map_) {
        map_->EraseMapPoint(shared_from_this());
    }
}

std::shared_ptr<MapPoint> MapPoint::GetReplaced() const {
    std::lock_guard<std::mutex> lock(mutex_features_);
    return replaced_.lock();
}

void MapPoint::SetBadFlag() {
    std::map<std::weak_ptr<KeyFrame>, int,
             std::owner_less<std::weak_ptr<KeyFrame>>>
        observations;
    {
        std::lock_guard<std::mutex> lock(mutex_features_);
        bad_ = true;
        observations = observations_;
        observations_.clear();
        observation_count_ = 0;
    }
    for (const auto& [weak_kf, index] : observations) {
        if (auto keyframe = weak_kf.lock()) {
            keyframe->EraseMapPointMatch(index);
        }
    }
    if (map_) {
        map_->EraseMapPoint(shared_from_this());
    }
}

bool MapPoint::isBad() const {
    std::lock_guard<std::mutex> lock(mutex_features_);
    return bad_;
}

void MapPoint::ComputeDistinctiveDescriptors() {
    std::vector<cv::Mat> descriptors;
    {
        std::lock_guard<std::mutex> lock(mutex_features_);
        if (bad_) {
            return;
        }
        for (const auto& [weak_keyframe, index] : observations_) {
            auto keyframe = weak_keyframe.lock();
            if (!keyframe || keyframe->isBad()) {
                continue;
            }
            const cv::Mat all = keyframe->GetDescriptors();
            if (index >= 0 && index < all.rows) {
                descriptors.push_back(all.row(index).clone());
            }
        }
    }
    if (descriptors.empty()) {
        return;
    }

    // Least median Hamming distance to the rest (ORB-SLAM3).
    const size_t N = descriptors.size();
    std::vector<std::vector<int>> distances(N, std::vector<int>(N, 0));
    for (size_t i = 0; i < N; ++i) {
        for (size_t j = i + 1; j < N; ++j) {
            const int d = feature::OrbMatcher::DescriptorDistance(
                descriptors[i], descriptors[j]);
            distances[i][j] = d;
            distances[j][i] = d;
        }
    }

    int best_median = INT_MAX;
    size_t best_idx = 0;
    for (size_t i = 0; i < N; ++i) {
        std::vector<int> row = distances[i];
        std::sort(row.begin(), row.end());
        const int median = row[(N - 1) / 2];
        if (median < best_median) {
            best_median = median;
            best_idx = i;
        }
    }

    std::lock_guard<std::mutex> lock(mutex_features_);
    descriptor_ = descriptors[best_idx].clone();
}

void MapPoint::UpdateNormalAndDepth() {
    std::map<std::weak_ptr<KeyFrame>, int,
             std::owner_less<std::weak_ptr<KeyFrame>>>
        observations;
    std::shared_ptr<KeyFrame> ref;
    Vec3 position;
    {
        std::lock_guard<std::mutex> lock_pos(mutex_position_);
        std::lock_guard<std::mutex> lock_feat(mutex_features_);
        if (bad_) {
            return;
        }
        observations = observations_;
        position = position_world_;
        ref = reference_keyframe_.lock();
    }
    if (!ref || observations.empty()) {
        return;
    }

    Vec3 normal = Vec3::Zero();
    int n = 0;
    for (const auto& [weak_kf, index] : observations) {
        auto kf = weak_kf.lock();
        if (!kf || kf->isBad() || index < 0) {
            continue;
        }
        Vec3 Ow = kf->GetCameraCenter();
        if (kf->HasDualCameraIndex() && index >= kf->num_left) {
            Ow = kf->GetRightCameraCenter();
        }
        const Vec3 ni = position - Ow;
        const double norm = ni.norm();
        if (norm < 1e-9) {
            continue;
        }
        normal += ni / norm;
        ++n;
    }
    if (n == 0) {
        return;
    }
    normal /= static_cast<double>(n);

    const Vec3 PC = position - ref->GetCameraCenter();
    const float dist = static_cast<float>(PC.norm());
    int level = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_features_);
        const auto it = observations_.find(ref);
        if (it != observations_.end() && it->second >= 0) {
            level = ref->GetKeyPoint(it->second).octave;
        }
    }
    float scale = 1.f;
    if (level >= 0 &&
        level < static_cast<int>(ref->scale_factors.size())) {
        scale = ref->scale_factors[static_cast<size_t>(level)];
    }

    std::lock_guard<std::mutex> lock(mutex_position_);
    normal_ = normal;
    max_distance_ = dist * scale;
    min_distance_ =
        max_distance_ /
        (ref->scale_factors.empty()
             ? 1.f
             : ref->scale_factors[ref->scale_factors.size() - 1]);
}

cv::Mat MapPoint::GetDescriptor() const {
    std::lock_guard<std::mutex> lock(mutex_features_);
    return descriptor_.clone();
}

float MapPoint::GetMinDistanceInvariance() const {
    std::lock_guard<std::mutex> lock(mutex_position_);
    return 0.8f * min_distance_;
}

float MapPoint::GetMaxDistanceInvariance() const {
    std::lock_guard<std::mutex> lock(mutex_position_);
    return 1.2f * max_distance_;
}

int MapPoint::PredictScale(float current_dist, int scale_levels,
                           float log_scale_factor) const {
    float ratio = 1.f;
    {
        std::lock_guard<std::mutex> lock(mutex_position_);
        if (current_dist > 1e-6f) {
            ratio = max_distance_ / current_dist;
        }
    }
    if (log_scale_factor < 1e-6f || scale_levels <= 0) {
        return 0;
    }
    int n_scale = static_cast<int>(std::ceil(std::log(ratio) / log_scale_factor));
    if (n_scale < 0) {
        n_scale = 0;
    } else if (n_scale >= scale_levels) {
        n_scale = scale_levels - 1;
    }
    return n_scale;
}

float MapPoint::GetFoundRatio() const {
    std::lock_guard<std::mutex> lock(mutex_features_);
    if (visible_count_ == 0) {
        return 0.f;
    }
    return static_cast<float>(found_count_) /
           static_cast<float>(visible_count_);
}

void MapPoint::IncreaseVisible(int n) {
    std::lock_guard<std::mutex> lock(mutex_features_);
    visible_count_ += n;
}

void MapPoint::IncreaseFound(int n) {
    std::lock_guard<std::mutex> lock(mutex_features_);
    found_count_ += n;
}

void MapPoint::UpdateMap(Map* map) { map_ = map; }

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
