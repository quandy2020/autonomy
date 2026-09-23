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
 * @file keyframe.cpp
 * @brief `KeyFrame` implementation: pose, covisibility, BoW, grid, and IMU pose.
 *
 * Corresponds to a subset of ORB-SLAM3 `KeyFrame.cc`.
 */

#include "autonomy/localization/atlas/map/keyframe.hpp"

#include <algorithm>
#include <cmath>

#include "autonomy/localization/atlas/map/map.hpp"
#include "autonomy/localization/atlas/map/map_point.hpp"
#include "autonomy/localization/atlas/sensor/imu/pose.hpp"

namespace autonomy {
namespace localization {
namespace atlas {

long unsigned int KeyFrame::next_id = 0;

KeyFrame::KeyFrame(const tracking::Frame& frame, Map* map) : map_(map) {
    id = next_id++;
    frame_id = frame.id;
    timestamp = frame.timestamp;
    fx = frame.fx;
    fy = frame.fy;
    cx = frame.cx;
    cy = frame.cy;
    inv_fx = frame.inv_fx;
    inv_fy = frame.inv_fy;
    baseline_times_fx = frame.baseline_times_fx;
    baseline_meters = frame.baseline_meters;
    depth_threshold = frame.depth_threshold;
    keypoints_ = frame.keypoints_undistorted;
    descriptors_ = frame.descriptors.clone();
    depths_ = frame.depths;
    right_coordinate_ = frame.right_coordinate;
    map_points_ = frame.map_points;
    if (map_points_.size() < keypoints_.size()) {
        map_points_.resize(keypoints_.size(), nullptr);
    }
    scale_factor = frame.scale_factor;
    log_scale_factor = frame.log_scale_factor;
    scale_levels = frame.scale_levels;
    scale_factors = frame.scale_factors;
    inverse_scale_factors = frame.inverse_scale_factors;
    level_sigma2 = frame.level_sigma2;
    inverse_level_sigma2 = frame.inverse_level_sigma2;
    if (frame.has_pose()) {
        pose_camera_world_ = frame.GetPose();
    }
    if (frame.HasBoW()) {
        bow_vector_ = frame.bow_vector();
        feat_vector_ = frame.feat_vector();
        bow_ready_ = true;
    }
    imu_bias = frame.imu_bias;
    imu_calib = frame.imu_calib;
    velocity_world = frame.velocity_world;
    has_velocity = frame.has_velocity;
    camera = frame.camera;
    camera2 = frame.camera2;
    T_c1_c2 = frame.T_c1_c2;
    left_to_right_match = frame.left_to_right_match;
    num_left = frame.num_left;
    num_right = frame.num_right;
    keypoints_right_ = frame.keypoints_right;
    if (frame.HasDualCameraIndex()) {
        const size_t n_total = static_cast<size_t>(frame.TotalFeatures());
        if (map_points_.size() < n_total) {
            map_points_.resize(n_total, nullptr);
        }
    }
    AssignFeaturesToGrid();
}

cv::KeyPoint KeyFrame::GetKeyPoint(int index) const {
    if (num_left >= 0 && index >= num_left) {
        const int ri = index - num_left;
        if (ri >= 0 && ri < static_cast<int>(keypoints_right_.size())) {
            return keypoints_right_[static_cast<size_t>(ri)];
        }
        return {};
    }
    if (index >= 0 && index < static_cast<int>(keypoints_.size())) {
        return keypoints_[static_cast<size_t>(index)];
    }
    return {};
}

void KeyFrame::SetPose(const SE3& pose_camera_world) {
    std::lock_guard<std::mutex> lock(mutex_pose_);
    pose_camera_world_ = pose_camera_world;
}

SE3 KeyFrame::GetPose() const {
    std::lock_guard<std::mutex> lock(mutex_pose_);
    return pose_camera_world_;
}

SE3 KeyFrame::GetPoseInverse() const {
    std::lock_guard<std::mutex> lock(mutex_pose_);
    return pose_camera_world_.inverse();
}

SE3 KeyFrame::GetImuPose() const {
    std::lock_guard<std::mutex> lock(mutex_pose_);
    return sensor::imu::CameraPoseToImuPose(
        pose_camera_world_, sensor::imu::DefaultCalibOr(imu_calib));
}

void KeyFrame::SetImuPose(const SE3& T_wb) {
    std::lock_guard<std::mutex> lock(mutex_pose_);
    pose_camera_world_ = sensor::imu::ImuPoseToCameraPose(
        T_wb, sensor::imu::DefaultCalibOr(imu_calib));
}

Vec3 KeyFrame::GetCameraCenter() const {
    std::lock_guard<std::mutex> lock(mutex_pose_);
    return pose_camera_world_.inverse().translation();
}

void KeyFrame::AddMapPoint(const std::shared_ptr<MapPoint>& map_point,
                           int index) {
    std::lock_guard<std::mutex> lock(mutex_features_);
    if (index < 0) {
        return;
    }
    if (static_cast<size_t>(index) >= map_points_.size()) {
        map_points_.resize(static_cast<size_t>(index) + 1, nullptr);
    }
    map_points_[static_cast<size_t>(index)] = map_point;
}

void KeyFrame::EraseMapPointMatch(int index) {
    std::lock_guard<std::mutex> lock(mutex_features_);
    if (index >= 0 && static_cast<size_t>(index) < map_points_.size()) {
        map_points_[static_cast<size_t>(index)].reset();
    }
}

void KeyFrame::EraseMapPointMatch(const std::shared_ptr<MapPoint>& map_point) {
    if (!map_point) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_features_);
    for (auto& slot : map_points_) {
        if (slot == map_point) {
            slot.reset();
        }
    }
}

void KeyFrame::ReplaceMapPointMatch(
    int index, const std::shared_ptr<MapPoint>& map_point) {
    AddMapPoint(map_point, index);
}

std::shared_ptr<MapPoint> KeyFrame::GetMapPoint(int index) const {
    std::lock_guard<std::mutex> lock(mutex_features_);
    if (index < 0 || static_cast<size_t>(index) >= map_points_.size()) {
        return nullptr;
    }
    return map_points_[static_cast<size_t>(index)];
}

std::vector<std::shared_ptr<MapPoint>> KeyFrame::GetMapPoints() const {
    std::lock_guard<std::mutex> lock(mutex_features_);
    return map_points_;
}

float KeyFrame::GetDepth(int index) const {
    if (index < 0 || static_cast<size_t>(index) >= depths_.size()) {
        return -1.f;
    }
    return depths_[static_cast<size_t>(index)];
}

void KeyFrame::SetVocabulary(
    const std::shared_ptr<feature::OrbVocabulary>& vocabulary) {
    vocabulary_ = vocabulary;
}

void KeyFrame::ComputeBoW() {
    if (bow_ready_) {
        return;
    }
    if (!vocabulary_ || !vocabulary_->is_valid() || descriptors_.empty()) {
        return;
    }
    vocabulary_->Transform(descriptors_, &bow_vector_, &feat_vector_);
    bow_ready_ = !bow_vector_.empty();
}

void KeyFrame::SetBoW(const fbow::BoWVector& bow,
                      const fbow::BoWFeatVector& feat) {
    bow_vector_ = bow;
    feat_vector_ = feat;
    bow_ready_ = !bow_vector_.empty();
}

void KeyFrame::AddConnection(const std::shared_ptr<KeyFrame>& keyframe,
                             int weight) {
    if (!keyframe || keyframe.get() == this) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(mutex_connections_);
        const auto it = connected_keyframe_weights_.find(keyframe);
        if (it != connected_keyframe_weights_.end() && it->second == weight) {
            return;
        }
        connected_keyframe_weights_[keyframe] = weight;
    }
    UpdateBestCovisibles();
}

void KeyFrame::EraseConnection(const std::shared_ptr<KeyFrame>& keyframe) {
    if (!keyframe) {
        return;
    }
    {
        std::lock_guard<std::mutex> lock(mutex_connections_);
        connected_keyframe_weights_.erase(keyframe);
    }
    UpdateBestCovisibles();
}

void KeyFrame::UpdateBestCovisibles() {
    std::lock_guard<std::mutex> lock(mutex_connections_);
    std::vector<std::pair<int, std::shared_ptr<KeyFrame>>> pairs;
    pairs.reserve(connected_keyframe_weights_.size());
    for (const auto& [weak_kf, weight] : connected_keyframe_weights_) {
        auto keyframe = weak_kf.lock();
        if (!keyframe || keyframe->isBad()) {
            continue;
        }
        pairs.emplace_back(weight, keyframe);
    }
    std::sort(pairs.begin(), pairs.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });
    ordered_connected_keyframes_.clear();
    ordered_weights_.clear();
    for (auto it = pairs.rbegin(); it != pairs.rend(); ++it) {
        ordered_connected_keyframes_.push_back(it->second);
        ordered_weights_.push_back(it->first);
    }
}

void KeyFrame::UpdateConnections(bool update_parent) {
    std::map<std::shared_ptr<KeyFrame>, int> counter;
    const auto map_points = GetMapPoints();
    for (const auto& map_point : map_points) {
        if (!map_point || map_point->isBad()) {
            continue;
        }
        for (const auto& [weak_kf, index] : map_point->GetObservations()) {
            (void)index;
            auto other = weak_kf.lock();
            if (!other || other.get() == this || other->isBad()) {
                continue;
            }
            if (other->GetMap() != map_) {
                continue;
            }
            ++counter[other];
        }
    }
    if (counter.empty()) {
        return;
    }

    int nmax = 0;
    std::shared_ptr<KeyFrame> kf_max;
    constexpr int kThreshold = 15;
    std::vector<std::pair<int, std::shared_ptr<KeyFrame>>> pairs;
    pairs.reserve(counter.size());
    for (const auto& [keyframe, weight] : counter) {
        if (weight > nmax) {
            nmax = weight;
            kf_max = keyframe;
        }
        if (weight >= kThreshold) {
            pairs.emplace_back(weight, keyframe);
            keyframe->AddConnection(shared_from_this(), weight);
        }
    }
    if (pairs.empty() && kf_max) {
        pairs.emplace_back(nmax, kf_max);
        kf_max->AddConnection(shared_from_this(), nmax);
    }

    std::sort(pairs.begin(), pairs.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });

    {
        std::lock_guard<std::mutex> lock(mutex_connections_);
        connected_keyframe_weights_.clear();
        for (const auto& [keyframe, weight] : counter) {
            connected_keyframe_weights_[keyframe] = weight;
        }
        ordered_connected_keyframes_.clear();
        ordered_weights_.clear();
        for (auto it = pairs.rbegin(); it != pairs.rend(); ++it) {
            ordered_connected_keyframes_.push_back(it->second);
            ordered_weights_.push_back(it->first);
        }
        if (update_parent && first_connection_ && !ordered_connected_keyframes_.empty()) {
            parent_ = ordered_connected_keyframes_.front();
            if (auto parent = parent_.lock()) {
                parent->AddChild(shared_from_this());
            }
            first_connection_ = false;
        }
    }
}

std::vector<std::shared_ptr<KeyFrame>> KeyFrame::GetBestCovisibilityKeyFrames(
    int n) const {
    std::lock_guard<std::mutex> lock(mutex_connections_);
    if (ordered_connected_keyframes_.size() < static_cast<size_t>(n)) {
        return ordered_connected_keyframes_;
    }
    return {ordered_connected_keyframes_.begin(),
            ordered_connected_keyframes_.begin() + n};
}

std::vector<std::shared_ptr<KeyFrame>> KeyFrame::GetCovisiblesByWeight(
    int weight) const {
    std::lock_guard<std::mutex> lock(mutex_connections_);
    std::vector<std::shared_ptr<KeyFrame>> result;
    for (size_t i = 0; i < ordered_connected_keyframes_.size(); ++i) {
        if (ordered_weights_[i] < weight) {
            break;
        }
        result.push_back(ordered_connected_keyframes_[i]);
    }
    return result;
}

std::map<std::weak_ptr<KeyFrame>, int, std::owner_less<std::weak_ptr<KeyFrame>>>
KeyFrame::GetConnectedKeyFrames() const {
    std::lock_guard<std::mutex> lock(mutex_connections_);
    return connected_keyframe_weights_;
}

void KeyFrame::AddChild(const std::shared_ptr<KeyFrame>& child) {
    if (!child) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_connections_);
    children_.insert(child);
}

void KeyFrame::ChangeParent(const std::shared_ptr<KeyFrame>& parent) {
    {
        std::lock_guard<std::mutex> lock(mutex_connections_);
        parent_ = parent;
    }
    if (parent) {
        parent->AddChild(shared_from_this());
    }
}

void KeyFrame::EraseChild(const std::shared_ptr<KeyFrame>& child) {
    if (!child) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_connections_);
    children_.erase(child);
}

std::shared_ptr<KeyFrame> KeyFrame::GetParent() const {
    std::lock_guard<std::mutex> lock(mutex_connections_);
    return parent_.lock();
}

std::set<std::shared_ptr<KeyFrame>> KeyFrame::GetChildren() const {
    std::lock_guard<std::mutex> lock(mutex_connections_);
    std::set<std::shared_ptr<KeyFrame>> children;
    for (const auto& child : children_) {
        if (auto locked = child.lock()) {
            children.insert(std::move(locked));
        }
    }
    return children;
}

void KeyFrame::SetBadFlag() {
    std::shared_ptr<KeyFrame> parent;
    {
        std::lock_guard<std::mutex> lock(mutex_connections_);
        parent = parent_.lock();
    }
    SE3 to_parent = SE3Identity();
    if (parent) {
        to_parent = GetPose() * parent->GetPoseInverse();
    }
    std::lock_guard<std::mutex> lock(mutex_connections_);
    if (parent) {
        pose_to_parent = to_parent;
    }
    bad_ = true;
}

bool KeyFrame::isBad() const {
    std::lock_guard<std::mutex> lock(mutex_connections_);
    return bad_;
}

void KeyFrame::AddLoopEdge(const std::shared_ptr<KeyFrame>& keyframe) {
    if (!keyframe) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_connections_);
    loop_edges_.insert(keyframe);
}

void KeyFrame::AddMergeEdge(const std::shared_ptr<KeyFrame>& keyframe) {
    if (!keyframe) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_connections_);
    merge_edges_.insert(keyframe);
}

std::set<std::shared_ptr<KeyFrame>> KeyFrame::GetLoopEdges() const {
    std::lock_guard<std::mutex> lock(mutex_connections_);
    std::set<std::shared_ptr<KeyFrame>> edges;
    for (const auto& weak : loop_edges_) {
        if (auto kf = weak.lock()) {
            edges.insert(kf);
        }
    }
    return edges;
}

std::set<std::shared_ptr<KeyFrame>> KeyFrame::GetMergeEdges() const {
    std::lock_guard<std::mutex> lock(mutex_connections_);
    std::set<std::shared_ptr<KeyFrame>> edges;
    for (const auto& weak : merge_edges_) {
        if (auto kf = weak.lock()) {
            edges.insert(kf);
        }
    }
    return edges;
}

void KeyFrame::UpdateMap(Map* map) { map_ = map; }

bool KeyFrame::IsInImage(float x, float y) const {
    return x >= min_x_ && x < max_x_ && y >= min_y_ && y < max_y_;
}

void KeyFrame::AssignFeaturesToGrid() {
    min_x_ = 0.f;
    min_y_ = 0.f;
    max_x_ = 0.f;
    max_y_ = 0.f;
    for (const auto& keypoint : keypoints_) {
        max_x_ = std::max(max_x_, keypoint.pt.x);
        max_y_ = std::max(max_y_, keypoint.pt.y);
    }
    for (const auto& keypoint : keypoints_right_) {
        max_x_ = std::max(max_x_, keypoint.pt.x);
        max_y_ = std::max(max_y_, keypoint.pt.y);
    }
    if (max_x_ <= min_x_ || max_y_ <= min_y_) {
        max_x_ = static_cast<float>(std::max(1, static_cast<int>(max_x_) + 1));
        max_y_ = static_cast<float>(std::max(1, static_cast<int>(max_y_) + 1));
    }
    grid_element_width_inv_ =
        static_cast<float>(kGridCols) / (max_x_ - min_x_);
    grid_element_height_inv_ =
        static_cast<float>(kGridRows) / (max_y_ - min_y_);
    for (int i = 0; i < kGridCols; ++i) {
        for (int j = 0; j < kGridRows; ++j) {
            grid_[i][j].clear();
            grid_right_[i][j].clear();
        }
    }
    const size_t n_left =
        HasDualCameraIndex() ? static_cast<size_t>(num_left) : keypoints_.size();
    for (size_t i = 0; i < n_left && i < keypoints_.size(); ++i) {
        const int pos_x = static_cast<int>(
            std::floor((keypoints_[i].pt.x - min_x_) * grid_element_width_inv_));
        const int pos_y = static_cast<int>(std::floor(
            (keypoints_[i].pt.y - min_y_) * grid_element_height_inv_));
        if (pos_x >= 0 && pos_x < kGridCols && pos_y >= 0 && pos_y < kGridRows) {
            grid_[pos_x][pos_y].push_back(i);
        }
    }
    if (HasDualCameraIndex()) {
        for (size_t i = 0; i < keypoints_right_.size(); ++i) {
            const int pos_x = static_cast<int>(std::floor(
                (keypoints_right_[i].pt.x - min_x_) * grid_element_width_inv_));
            const int pos_y = static_cast<int>(std::floor(
                (keypoints_right_[i].pt.y - min_y_) *
                grid_element_height_inv_));
            if (pos_x >= 0 && pos_x < kGridCols && pos_y >= 0 &&
                pos_y < kGridRows) {
                grid_right_[pos_x][pos_y].push_back(i);
            }
        }
    }
}

std::vector<size_t> KeyFrame::GetFeaturesInArea(float x, float y, float radius,
                                                bool right) const {
    std::vector<size_t> indices;
    const int min_cell_x = std::max(
        0, static_cast<int>(
               std::floor((x - radius - min_x_) * grid_element_width_inv_)));
    const int max_cell_x = std::min(
        kGridCols - 1,
        static_cast<int>(
            std::floor((x + radius - min_x_) * grid_element_width_inv_)));
    const int min_cell_y = std::max(
        0, static_cast<int>(
               std::floor((y - radius - min_y_) * grid_element_height_inv_)));
    const int max_cell_y = std::min(
        kGridRows - 1,
        static_cast<int>(
            std::floor((y + radius - min_y_) * grid_element_height_inv_)));
    for (int ix = min_cell_x; ix <= max_cell_x; ++ix) {
        for (int iy = min_cell_y; iy <= max_cell_y; ++iy) {
            const auto& cell = right ? grid_right_[ix][iy] : grid_[ix][iy];
            for (const size_t index : cell) {
                const cv::KeyPoint& kp =
                    right ? keypoints_right_[index] : keypoints_[index];
                const float dx = kp.pt.x - x;
                const float dy = kp.pt.y - y;
                if (dx * dx + dy * dy < radius * radius) {
                    indices.push_back(index);
                }
            }
        }
    }
    return indices;
}

SE3 KeyFrame::GetRightPose() const {
    // ORB: mTrl * mTcw; Trl = T_c1_c2^{-1}.
    return T_c1_c2.inverse() * GetPose();
}

SE3 KeyFrame::GetRightPoseInverse() const {
    // ORB: mTwc * mTlr.
    return GetPose().inverse() * T_c1_c2;
}

Vec3 KeyFrame::GetRightCameraCenter() const {
    // ORB: (mTwc * mTlr).translation().
    return GetRightPoseInverse().translation();
}

float KeyFrame::ComputeSceneMedianDepth(int q) const {
    const auto map_points = GetMapPoints();
    const Vec3 Ow = GetCameraCenter();
    const SE3 Tcw = GetPose();
    std::vector<float> depths;
    depths.reserve(map_points.size());
    for (const auto& map_point : map_points) {
        if (!map_point || map_point->isBad()) {
            continue;
        }
        const Vec3 Pc = Tcw * map_point->GetWorldPos();
        if (Pc.z() > 0.0) {
            depths.push_back(static_cast<float>(Pc.z()));
        }
    }
    if (depths.empty()) {
        (void)Ow;
        return -1.f;
    }
    std::sort(depths.begin(), depths.end());
    return depths[(depths.size() - 1) / std::max(1, q)];
}

int KeyFrame::TrackedMapPoints(int min_observations) const {
    int count = 0;
    const auto matches = GetMapPointMatches();
    for (const auto& map_point : matches) {
        if (!map_point || map_point->isBad()) {
            continue;
        }
        if (map_point->Observations() >= min_observations) {
            ++count;
        }
    }
    return count;
}

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
