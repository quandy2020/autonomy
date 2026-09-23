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
 * @file local_mapping.cpp
 * @brief `LocalMapping` implementation: queue consume, triangulate, fuse, cull, IMU init.
 *
 * Corresponds to ORB-SLAM3 `LocalMapping.cc`.
 */

#include "autonomy/localization/atlas/map/local_mapping.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

#include "autonomy/localization/atlas/backend/loop_closing.hpp"
#include "autonomy/localization/atlas/backend/optimizer.hpp"
#include "autonomy/localization/atlas/common/geometric_tools.hpp"
#include "autonomy/localization/atlas/frontend/match/orb_matcher.hpp"
#include "autonomy/localization/atlas/frontend/tracking/tracker.hpp"
#include "autonomy/localization/atlas/map/map_point.hpp"
#include "autonomy/localization/atlas/system/slam_scheduler.hpp"

namespace autonomy {
namespace localization {
namespace atlas {

LocalMapping::LocalMapping(Map* map, bool monocular)
    : map_(map), monocular_(monocular) {}

void LocalMapping::SetLoopCloser(backend::LoopClosing* loop_closing) {
    loop_closing_ = loop_closing;
}

void LocalMapping::InsertKeyFrame(const std::shared_ptr<KeyFrame>& keyframe) {
    if (!keyframe || stop_requested_.load()) {
        return;
    }
    keyframe_queue_.Enqueue(keyframe);
}

bool LocalMapping::ProcessNextKeyFrame() {
    if (stop_requested_.load() || pause_requested_.load()) {
        if (pause_requested_.load()) {
            stopped_.store(true);
        }
        return false;
    }
    std::shared_ptr<KeyFrame> keyframe;
    if (!keyframe_queue_.Dequeue(&keyframe) || !keyframe) {
        return false;
    }
    ProcessPipeline(keyframe);
    return true;
}

void LocalMapping::EmptyQueue() {
    // ORB EmptyQueue: only ProcessNewKeyFrame (no BA / triangulation / loop insert).
    std::shared_ptr<KeyFrame> keyframe;
    while (keyframe_queue_.Dequeue(&keyframe)) {
        if (!keyframe) {
            continue;
        }
        ProcessNewKeyFrame(keyframe);
    }
}

int LocalMapping::KeyframesInQueue() const {
    return static_cast<int>(
        const_cast<::autolink::base::ThreadSafeQueue<std::shared_ptr<KeyFrame>>*>(
            &keyframe_queue_)
            ->Size());
}

void LocalMapping::Start(SlamScheduler* scheduler) {
    if (scheduler == nullptr || async_.load()) {
        return;
    }
    stop_requested_.store(false);
    pause_requested_.store(false);
    stopped_.store(false);
    async_.store(true);
    worker_future_ = scheduler->Enqueue([this] { Run(); });
}

void LocalMapping::Run() {
    while (!stop_requested_.load()) {
        if (pause_requested_.load() && !not_stop_.load()) {
            stopped_.store(true);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        stopped_.store(false);

        std::shared_ptr<KeyFrame> keyframe;
        if (!keyframe_queue_.Dequeue(&keyframe)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        if (!keyframe || stop_requested_.load()) {
            continue;
        }
        if (pause_requested_.load() && !not_stop_.load()) {
            keyframe_queue_.Enqueue(keyframe);
            continue;
        }
        ProcessPipeline(keyframe);
    }
    stopped_.store(true);
    async_.store(false);
}

void LocalMapping::RequestPause() {
    if (not_stop_.load()) {
        return;
    }
    pause_requested_.store(true);
    abort_ba_.store(true);
}

void LocalMapping::Release() {
    if (stop_requested_.load()) {
        return;
    }
    pause_requested_.store(false);
    not_stop_.store(false);
    stopped_.store(false);
}

bool LocalMapping::SetNotStop(bool flag) {
    if (flag && stopped_.load()) {
        return false;
    }
    not_stop_.store(flag);
    return true;
}

void LocalMapping::RequestStop() {
    stop_requested_.store(true);
    pause_requested_.store(true);
    keyframe_queue_.BreakAllWait();
    if (worker_future_.valid()) {
        worker_future_.wait();
    }
    stopped_.store(true);
    async_.store(false);
}

void LocalMapping::ProcessPipeline(const std::shared_ptr<KeyFrame>& keyframe) {
    ProcessNewKeyFrame(keyframe);
    MapPointCulling();
    CreateNewMapPoints();
    SearchInNeighbors();
    const bool skip_ba = abort_ba_.exchange(false);
    if (!skip_ba) {
        if (current_keyframe_ && current_keyframe_->imu_preintegrated) {
            if (map_ && !map_->isImuInitialized() &&
                map_->KeyFramesInMap() >= 10) {
                InitializeImu();
            }
            if (map_ && map_->isImuInitialized()) {
                auto previous = current_keyframe_->previous_keyframe.lock();
                auto previous2 =
                    previous ? previous->previous_keyframe.lock() : nullptr;
                if (previous && previous2 && !map_->GetInertialBA2()) {
                    const double dist =
                        (previous->GetCameraCenter() -
                         current_keyframe_->GetCameraCenter())
                            .norm() +
                        (previous2->GetCameraCenter() -
                         previous->GetCameraCenter())
                            .norm();
                    if (dist > 0.05) {
                        init_motion_time_ +=
                            current_keyframe_->timestamp - previous->timestamp;
                    }
                    if (init_motion_time_ < 10.0 && dist < 0.02) {
                        bad_imu_.store(true);
                    }
                }
                const int inliers =
                    tracker_ != nullptr ? tracker_->matches_inliers() : 0;
                const bool large = (monocular_ && inliers > 75) ||
                                   (!monocular_ && inliers > 100);
                const bool reinit = !map_->GetInertialBA2();
                backend::Optimizer::LocalInertialBA(current_keyframe_, map_,
                                                    nullptr, large, reinit);
                // VIBA1 @ t>5s, VIBA2 @ t>15s (ORB-SLAM3 LocalMapping).
                if (imu_init_timestamp_ >= 0.0 && current_keyframe_) {
                    const double t =
                        current_keyframe_->timestamp - imu_init_timestamp_;
                    if (!map_->GetInertialBA1() && t > 5.0) {
                        InitializeImu(1.f, 1e5f, true);
                        map_->SetInertialBA1(true);
                    } else if (!map_->GetInertialBA2() && t > 15.0) {
                        InitializeImu(0.f, 0.f, true);
                        map_->SetInertialBA2(true);
                    }
                }
                if (monocular_ && map_->KeyFramesInMap() <= 200) {
                    ScaleRefinement();
                }
            } else {
                backend::Optimizer::LocalBundleAdjustment(current_keyframe_,
                                                          map_);
            }
        } else {
            backend::Optimizer::LocalBundleAdjustment(current_keyframe_, map_);
        }
    }
    KeyFrameCulling();
    if (loop_closing_ && current_keyframe_) {
        loop_closing_->InsertKeyFrame(current_keyframe_);
    }
    if (map_) {
        map_->IncreaseChangeIndex();
    }
}

void LocalMapping::InitializeImu(float prior_g, float prior_a,
                                 bool run_full_inertial_ba) {
    if (!map_ || !current_keyframe_) {
        return;
    }
    const bool first_init = !map_->isImuInitialized();
    const double min_time = monocular_ ? 2.0 : 1.0;
    if (first_init) {
        if (map_->KeyFramesInMap() < 10) {
            return;
        }
        // Temporal chain length.
        auto kf = current_keyframe_;
        double first_ts = kf->timestamp;
        int n = 0;
        while (kf) {
            first_ts = kf->timestamp;
            ++n;
            kf = kf->previous_keyframe.lock();
        }
        if (n < 10 ||
            current_keyframe_->timestamp - first_ts < min_time) {
            return;
        }
        first_imu_keyframe_ts_ = first_ts;
    }

    initializing_.store(true);
    const auto t0 = std::chrono::steady_clock::now();
    Mat33 Rwg = Mat33::Identity();
    double scale = 1.0;
    Vec3 bg = Vec3::Zero();
    Vec3 ba = Vec3::Zero();
    if (!first_init && current_keyframe_) {
        bg = current_keyframe_->imu_bias.gyroscope;
        ba = current_keyframe_->imu_bias.accelerometer;
    }
    Eigen::MatrixXd covariance;
    backend::Optimizer::InertialOptimization(map_, &Rwg, &scale, &bg, &ba,
                                             monocular_, prior_g, prior_a,
                                             false, &covariance);
    const auto t1 = std::chrono::steady_clock::now();
    init_cost_sec_ = std::chrono::duration<double>(t1 - t0).count();
    gravity_rotation_ = Rwg;
    imu_scale_ = scale;
    gyro_bias_ = bg;
    acc_bias_ = ba;
    inertial_covariance_ = covariance;
    if (first_init) {
        ++init_section_;
        init_time_sec_ = current_keyframe_->timestamp - first_imu_keyframe_ts_;
    }
    if (scale < 0.1) {
        initializing_.store(false);
        return;
    }

    sensor::imu::Bias bias;
    bias.gyroscope = bg;
    bias.accelerometer = ba;
    if (std::fabs(scale - 1.0) > 1e-5 || !monocular_) {
        SE3 Twg = SE3Identity();
        Twg.linear() = Rwg.transpose();
        map_->ApplyScaledRotation(Twg, scale, true);
        if (tracker_) {
            tracker_->UpdateFrameIMU(static_cast<float>(scale), bias,
                                     current_keyframe_);
        }
    }
    if (tracker_) {
        tracker_->UpdateFrameIMU(1.0f, bias, current_keyframe_);
    }

    if (first_init) {
        map_->SetImuInitialized(true);
        auto all_kf = map_->GetAllKeyFrames();
        for (const auto& kf : all_kf) {
            if (kf) {
                kf->imu_ready = true;
                kf->imu_bias = bias;
            }
        }
        imu_init_timestamp_ = current_keyframe_->timestamp;
        last_scale_refine_slot_ = -1;
    } else if (current_keyframe_) {
        current_keyframe_->imu_bias = bias;
    }

    if (run_full_inertial_ba) {
        if (prior_a != 0.f) {
            backend::Optimizer::FullInertialBA(map_, 100, false, nullptr, true,
                                               prior_g, prior_a);
        } else {
            backend::Optimizer::FullInertialBA(map_, 100, false, nullptr,
                                               false);
        }
    }
    initializing_.store(false);
}

void LocalMapping::ScaleRefinement() {
    if (!map_ || !current_keyframe_ || !monocular_ ||
        !map_->isImuInitialized() || imu_init_timestamp_ < 0.0) {
        return;
    }
    const double t = current_keyframe_->timestamp - imu_init_timestamp_;
    // ORB-SLAM3 windows: 25, 35, ..., 75 s (±0.5).
    int slot = -1;
    for (int center = 25; center <= 75; center += 10) {
        if (t > center - 0.5 && t < center + 0.5) {
            slot = center;
            break;
        }
    }
    if (slot < 0 || slot == last_scale_refine_slot_) {
        return;
    }
    Mat33 Rwg = Mat33::Identity();
    double scale = 1.0;
    backend::Optimizer::ScaleRefinement(map_, &Rwg, &scale);
    if (scale < 0.1) {
        return;
    }
    if (std::fabs(scale - 1.0) > 0.002) {
        SE3 Twg = SE3Identity();
        Twg.linear() = Rwg.transpose();
        map_->ApplyScaledRotation(Twg, scale, true);
        if (tracker_) {
            tracker_->UpdateFrameIMU(static_cast<float>(scale),
                                     current_keyframe_->imu_bias,
                                     current_keyframe_);
        }
    }
    last_scale_refine_slot_ = slot;
}

void LocalMapping::ProcessNewKeyFrame(
    const std::shared_ptr<KeyFrame>& keyframe) {
    current_keyframe_ = keyframe;
    keyframe->ComputeBoW();

    const auto matches = keyframe->GetMapPointMatches();
    for (size_t i = 0; i < matches.size(); ++i) {
        const auto& map_point = matches[i];
        if (!map_point || map_point->isBad()) {
            continue;
        }
        if (!map_point->IsInKeyFrame(keyframe)) {
            map_point->AddObservation(keyframe, static_cast<int>(i));
            map_point->UpdateNormalAndDepth();
            map_point->ComputeDistinctiveDescriptors();
        } else {
            recent_map_points_.push_back(map_point);
        }
    }

    keyframe->UpdateConnections();
    if (map_ != nullptr) {
        map_->AddKeyFrame(keyframe);
        map_->SetReferenceMapPoints(map_->GetAllMapPoints());
    }
}

void LocalMapping::MapPointCulling() {
    if (!current_keyframe_) {
        return;
    }
    const long unsigned int current_id = current_keyframe_->id;
    const int obs_threshold = monocular_ ? 2 : 3;

    for (auto it = recent_map_points_.begin();
         it != recent_map_points_.end();) {
        const auto& map_point = *it;
        if (!map_point || map_point->isBad()) {
            it = recent_map_points_.erase(it);
        } else if (map_point->GetFoundRatio() < 0.25f) {
            map_point->SetBadFlag();
            it = recent_map_points_.erase(it);
        } else if (static_cast<int>(current_id - map_point->first_keyframe_id) >=
                       2 &&
                   map_point->Observations() <= obs_threshold) {
            map_point->SetBadFlag();
            it = recent_map_points_.erase(it);
        } else if (static_cast<int>(current_id - map_point->first_keyframe_id) >=
                   3) {
            it = recent_map_points_.erase(it);
        } else {
            ++it;
        }
    }
}

void LocalMapping::CreateNewMapPoints() {
    if (!current_keyframe_ || !map_) {
        return;
    }

    const int nn = monocular_ ? 20 : 10;
    auto neighbors =
        current_keyframe_->GetBestCovisibilityKeyFrames(nn);

    // Stereo / RGB-D: create close points from depth on current KF.
    if (!monocular_) {
        const auto depths = current_keyframe_->GetDepths();
        const auto keypoints = current_keyframe_->GetKeyPoints();
        const SE3 Twc = current_keyframe_->GetPoseInverse();
        int created = 0;
        for (size_t i = 0; i < keypoints.size() && created < 100; ++i) {
            if (current_keyframe_->GetMapPoint(static_cast<int>(i))) {
                continue;
            }
            if (i >= depths.size() || depths[i] <= 0.f) {
                continue;
            }
            if (depths[i] > current_keyframe_->depth_threshold && created > 50) {
                continue;
            }
            const float u = keypoints[i].pt.x;
            const float v = keypoints[i].pt.y;
            const float z = depths[i];
            const float x =
                (u - current_keyframe_->cx) * z * current_keyframe_->inv_fx;
            const float y =
                (v - current_keyframe_->cy) * z * current_keyframe_->inv_fy;
            const Vec3 Pw = Twc * Vec3(x, y, z);
            auto map_point =
                std::make_shared<MapPoint>(Pw, current_keyframe_, map_);
            map_point->AddObservation(current_keyframe_, static_cast<int>(i));
            current_keyframe_->AddMapPoint(map_point, static_cast<int>(i));
            map_point->ComputeDistinctiveDescriptors();
            map_point->UpdateNormalAndDepth();
            map_->AddMapPoint(map_point);
            recent_map_points_.push_back(map_point);
            ++created;
        }
    }

    // Triangulate with covisible neighbors (ORB-SLAM3 CreateNewMapPoints).
    feature::OrbMatcher matcher(0.6f, false);
    const SE3 Tcw1_left = current_keyframe_->GetPose();
    const Vec3 Ow1_left = current_keyframe_->GetCameraCenter();

    for (const auto& neighbor : neighbors) {
        if (!neighbor || neighbor->isBad()) {
            continue;
        }
        const Vec3 Ow2_left = neighbor->GetCameraCenter();
        const float baseline = static_cast<float>((Ow2_left - Ow1_left).norm());
        if (!monocular_) {
            if (baseline < neighbor->baseline_meters) {
                continue;
            }
        } else {
            const float median = neighbor->ComputeSceneMedianDepth(2);
            if (median > 0.f && baseline / median < 0.01f) {
                continue;
            }
        }

        std::vector<std::pair<size_t, size_t>> pairs;
        matcher.SearchForTriangulation(current_keyframe_, neighbor, &pairs,
                                       false);
        if (pairs.empty()) {
            continue;
        }

        const bool dual = current_keyframe_->HasDualCameraIndex() &&
                          neighbor->HasDualCameraIndex() &&
                          current_keyframe_->camera2 && neighbor->camera2;
        const SE3 Tcw2_left = neighbor->GetPose();

        for (const auto& [i1, i2] : pairs) {
            if (current_keyframe_->GetMapPoint(static_cast<int>(i1))) {
                continue;
            }
            const bool right1 =
                current_keyframe_->HasDualCameraIndex() &&
                static_cast<int>(i1) >= current_keyframe_->num_left;
            const bool right2 = neighbor->HasDualCameraIndex() &&
                                static_cast<int>(i2) >= neighbor->num_left;

            SE3 Tcw1 = Tcw1_left;
            SE3 Tcw2 = Tcw2_left;
            Vec3 Ow1 = Ow1_left;
            Vec3 Ow2 = Ow2_left;
            const sensor::GeometricCamera* cam1 =
                current_keyframe_->camera.get();
            const sensor::GeometricCamera* cam2 = neighbor->camera.get();
            if (dual) {
                if (right1 && right2) {
                    Tcw1 = current_keyframe_->GetRightPose();
                    Ow1 = current_keyframe_->GetRightCameraCenter();
                    Tcw2 = neighbor->GetRightPose();
                    Ow2 = neighbor->GetRightCameraCenter();
                    cam1 = current_keyframe_->camera2.get();
                    cam2 = neighbor->camera2.get();
                } else if (right1 && !right2) {
                    Tcw1 = current_keyframe_->GetRightPose();
                    Ow1 = current_keyframe_->GetRightCameraCenter();
                    cam1 = current_keyframe_->camera2.get();
                } else if (!right1 && right2) {
                    Tcw2 = neighbor->GetRightPose();
                    Ow2 = neighbor->GetRightCameraCenter();
                    cam2 = neighbor->camera2.get();
                }
            }

            const cv::KeyPoint kp1 =
                current_keyframe_->GetKeyPoint(static_cast<int>(i1));
            const cv::KeyPoint kp2 =
                neighbor->GetKeyPoint(static_cast<int>(i2));
            Vec3 Pw;
            if (!GeometricTools::TriangulatePixels(
                    kp1.pt, kp2.pt, cam1, cam2, Tcw1, Tcw2,
                    current_keyframe_->fx, current_keyframe_->fy,
                    current_keyframe_->cx, current_keyframe_->cy, neighbor->fx,
                    neighbor->fy, neighbor->cx, neighbor->cy, &Pw)) {
                continue;
            }
            const Vec3 Pc1 = Tcw1 * Pw;
            const Vec3 Pc2 = Tcw2 * Pw;
            if (Pc1.z() <= 0.0 || Pc2.z() <= 0.0) {
                continue;
            }
            const float dist1 = static_cast<float>((Pw - Ow1).norm());
            const float dist2 = static_cast<float>((Pw - Ow2).norm());
            if (far_points_ &&
                (dist1 >= th_far_points_ || dist2 >= th_far_points_)) {
                continue;
            }
            auto map_point =
                std::make_shared<MapPoint>(Pw, current_keyframe_, map_);
            map_point->AddObservation(current_keyframe_, static_cast<int>(i1));
            map_point->AddObservation(neighbor, static_cast<int>(i2));
            current_keyframe_->AddMapPoint(map_point, static_cast<int>(i1));
            neighbor->AddMapPoint(map_point, static_cast<int>(i2));
            map_point->ComputeDistinctiveDescriptors();
            map_point->UpdateNormalAndDepth();
            map_->AddMapPoint(map_point);
            recent_map_points_.push_back(map_point);
        }
    }
}

void LocalMapping::SearchInNeighbors() {
    if (!current_keyframe_) {
        return;
    }
    const int nn = monocular_ ? 30 : 10;
    auto neighbors = current_keyframe_->GetBestCovisibilityKeyFrames(nn);
    std::vector<std::shared_ptr<KeyFrame>> targets;
    for (const auto& neighbor : neighbors) {
        if (!neighbor || neighbor->isBad() ||
            neighbor->fuse_target_for_kf == current_keyframe_->id) {
            continue;
        }
        targets.push_back(neighbor);
        neighbor->fuse_target_for_kf = current_keyframe_->id;
    }
    for (size_t i = 0; i < targets.size(); ++i) {
        for (const auto& second :
             targets[i]->GetBestCovisibilityKeyFrames(20)) {
            if (!second || second->isBad() ||
                second->fuse_target_for_kf == current_keyframe_->id ||
                second->id == current_keyframe_->id) {
                continue;
            }
            targets.push_back(second);
            second->fuse_target_for_kf = current_keyframe_->id;
        }
    }

    feature::OrbMatcher matcher;
    const auto matches = current_keyframe_->GetMapPointMatches();
    for (const auto& target : targets) {
        matcher.Fuse(target, matches);
        if (target->HasDualCameraIndex()) {
            matcher.Fuse(target, matches, 3.f, true);
        }
    }

    std::vector<std::shared_ptr<MapPoint>> fuse_candidates;
    for (const auto& target : targets) {
        for (const auto& map_point : target->GetMapPointMatches()) {
            if (!map_point || map_point->isBad() ||
                map_point->fuse_candidate_for_kf == current_keyframe_->id) {
                continue;
            }
            map_point->fuse_candidate_for_kf = current_keyframe_->id;
            fuse_candidates.push_back(map_point);
        }
    }
    matcher.Fuse(current_keyframe_, fuse_candidates);
    if (current_keyframe_->HasDualCameraIndex()) {
        matcher.Fuse(current_keyframe_, fuse_candidates, 3.f, true);
    }

    for (const auto& map_point : current_keyframe_->GetMapPointMatches()) {
        if (!map_point || map_point->isBad()) {
            continue;
        }
        map_point->ComputeDistinctiveDescriptors();
        map_point->UpdateNormalAndDepth();
    }
    current_keyframe_->UpdateConnections();
}

void LocalMapping::KeyFrameCulling() {
    if (!current_keyframe_ || !map_ || map_->KeyFramesInMap() < 5) {
        return;
    }
    // ORB-SLAM3 KeyFrameCulling: redundant if >redundant_th of MPs are seen
    // in ≥3 other KFs at same or finer scale.
    current_keyframe_->UpdateBestCovisibles();
    const auto local_keyframes =
        current_keyframe_->GetBestCovisibilityKeyFrames(monocular_ ? 20 : 10);
    const bool imu_init = map_->isImuInitialized();
    // ORB: !mbInertial → 0.9; mono(+imu) → 0.9; stereo+imu → 0.5.
    // Atlas uses imu_init as stand-in for mbInertial once IMU is up.
    const float redundant_th =
        (!imu_init) ? 0.9f : (monocular_ ? 0.9f : 0.5f);
    constexpr int kThObs = 3;
    constexpr int kNd = 21;

    unsigned int last_id = 0;
    if (imu_init) {
        int count = 0;
        auto aux = current_keyframe_;
        while (count < kNd) {
            auto prev = aux->previous_keyframe.lock();
            if (!prev) {
                break;
            }
            aux = prev;
            ++count;
        }
        last_id = static_cast<unsigned int>(aux->id);
    }

    for (const auto& keyframe : local_keyframes) {
        if (!keyframe || keyframe->isBad() || keyframe->id == 0) {
            continue;
        }
        if (imu_init) {
            if (map_->KeyFramesInMap() <= kNd) {
                continue;
            }
            if (keyframe->id + 2 > current_keyframe_->id) {
                continue;
            }
        }

        const auto map_points = keyframe->GetMapPointMatches();
        int redundant = 0;
        int total = 0;
        for (size_t i = 0; i < map_points.size(); ++i) {
            const auto& map_point = map_points[i];
            if (!map_point || map_point->isBad()) {
                continue;
            }
            if (!monocular_) {
                const float d = keyframe->GetDepth(static_cast<int>(i));
                if (d < 0.f || d > keyframe->depth_threshold) {
                    continue;
                }
            }
            ++total;
            if (map_point->Observations() <= kThObs) {
                continue;
            }
            const int scale_level =
                keyframe->GetKeyPoint(static_cast<int>(i)).octave;
            int n_obs = 0;
            for (const auto& [weak_kf, idx] : map_point->GetObservations()) {
                auto other = weak_kf.lock();
                if (!other || other.get() == keyframe.get() || other->isBad()) {
                    continue;
                }
                if (idx < 0 || idx >= other->TotalFeatures()) {
                    continue;
                }
                if (other->GetKeyPoint(idx).octave <= scale_level + 1) {
                    ++n_obs;
                    if (n_obs > kThObs) {
                        break;
                    }
                }
            }
            if (n_obs > kThObs) {
                ++redundant;
            }
        }
        if (total <= 0 || redundant <= redundant_th * total) {
            continue;
        }

        if (imu_init) {
            auto prev = keyframe->previous_keyframe.lock();
            auto next = keyframe->next_keyframe.lock();
            if (prev && next && keyframe->imu_preintegrated &&
                next->imu_preintegrated) {
                const double t = next->timestamp - prev->timestamp;
                const bool merge_window =
                    (keyframe->id < last_id && t < 3.0) || t < 0.5;
                const Vec3 dp = keyframe->GetImuPose().translation() -
                                prev->GetImuPose().translation();
                const bool merge_pre_ba2 =
                    !map_->GetInertialBA2() && dp.norm() < 0.02 && t < 3.0;
                if (merge_window || merge_pre_ba2) {
                    // ORB-SLAM3: MergePrevious then rewire temporal links.
                    next->imu_preintegrated->MergePrevious(
                        *keyframe->imu_preintegrated);
                    next->previous_keyframe = prev;
                    prev->next_keyframe = next;
                    keyframe->next_keyframe.reset();
                    keyframe->previous_keyframe.reset();
                    keyframe->SetBadFlag();
                    map_->EraseKeyFrame(keyframe);
                }
            } else if (!keyframe->imu_preintegrated ||
                       current_keyframe_->timestamp - keyframe->timestamp >
                           3.0) {
                keyframe->SetBadFlag();
                map_->EraseKeyFrame(keyframe);
            }
        } else {
            keyframe->SetBadFlag();
            map_->EraseKeyFrame(keyframe);
        }
    }
}

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
