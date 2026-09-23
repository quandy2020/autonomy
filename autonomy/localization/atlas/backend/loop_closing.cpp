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
 * @file loop_closing.cpp
 * @brief LoopClosing implementation: covisibility checks, Sim3 correction, map merge, async GBA.
 */

#include "autonomy/localization/atlas/backend/loop_closing.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <set>
#include <thread>
#include <unordered_map>
#include <unordered_set>

#include "autonomy/localization/atlas/backend/optimizer.hpp"
#include "autonomy/localization/atlas/backend/sim3_solver.hpp"
#include "autonomy/localization/atlas/frontend/match/orb_matcher.hpp"
#include "autonomy/localization/atlas/map/local_mapping.hpp"
#include "autonomy/localization/atlas/system/slam_scheduler.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
namespace backend {
namespace {

constexpr int kMinMapKeyFrames = 12;
constexpr int kBoWMatches = 20;
constexpr int kBoWInliers = 15;
constexpr int kSim3Inliers = 20;
constexpr int kProjMatches = 50;
constexpr int kProjOptMatches = 80;
constexpr int kRefineProjMatches = 30;
constexpr int kRefineOptMatches = 50;
constexpr int kRefineProjMatchesRep = 100;
constexpr int kCoincidenceThresh = 3;
constexpr int kNumCovisibles = 10;

SE3 Sim3ToSe3(const Sim3& sim3) {
    SE3 pose = SE3Identity();
    pose.linear() = sim3.rotation;
    pose.translation() = sim3.translation;
    return pose;
}

/** Map current-world geometry by \(S_{yw}\) (ORB `Map::ApplyScaledRotation`). */
void ApplySim3ToMap(Map* map, const Sim3& transform, bool scale_velocity) {
    if (!map) {
        return;
    }
    for (const auto& keyframe : map->GetAllKeyFrames()) {
        if (!keyframe) {
            continue;
        }
        const SE3 Twc = keyframe->GetPose().inverse();
        SE3 transformed = SE3Identity();
        transformed.linear() = transform.rotation * Twc.rotation();
        transformed.translation() =
            transform.rotation * (transform.scale * Twc.translation()) +
            transform.translation;
        keyframe->SetPose(transformed.inverse());
        if (keyframe->has_velocity) {
            keyframe->velocity_world =
                transform.rotation * keyframe->velocity_world;
            if (scale_velocity) {
                keyframe->velocity_world *= transform.scale;
            }
        }
    }
    for (const auto& map_point : map->GetAllMapPoints()) {
        if (!map_point || map_point->isBad()) {
            continue;
        }
        map_point->SetWorldPos(transform.Map(map_point->GetWorldPos()));
        map_point->UpdateNormalAndDepth();
    }
}

}  // namespace

LoopClosing::LoopClosing(Map* map, KeyFrameDatabase* keyframe_database,
                         MultiMap* multi_map)
    : map_(map),
      multi_map_(multi_map),
      keyframe_database_(keyframe_database) {}

void LoopClosing::InsertKeyFrame(const std::shared_ptr<KeyFrame>& keyframe) {
    if (!keyframe || keyframe->isBad() || finish_requested_.load()) {
        return;
    }
    keyframe_queue_.Enqueue(keyframe);
}

void LoopClosing::Start(SlamScheduler* scheduler) {
    if (scheduler == nullptr || async_.load()) {
        return;
    }
    scheduler_ = scheduler;
    finish_requested_.store(false);
    finished_.store(false);
    async_.store(true);
    worker_future_ = scheduler->Enqueue([this] { Run(); });
}

void LoopClosing::Run() {
    while (!finish_requested_.load()) {
        std::shared_ptr<KeyFrame> keyframe;
        if (!keyframe_queue_.WaitDequeue(&keyframe)) {
            break;
        }
        if (!keyframe || finish_requested_.load()) {
            continue;
        }
        ProcessKeyFrame(keyframe);
    }
    finished_.store(true);
    async_.store(false);
}

void LoopClosing::ResetLoopState() {
    loop_num_coincidences_ = 0;
    loop_num_not_found_ = 0;
    loop_matched_kf_.reset();
    loop_last_current_kf_.reset();
    loop_sim3_slw_ = Sim3::Identity();
    loop_map_points_.clear();
    loop_matched_mps_.clear();
}

void LoopClosing::ResetMergeState() {
    merge_num_coincidences_ = 0;
    merge_num_not_found_ = 0;
    merge_matched_kf_.reset();
    merge_last_current_kf_.reset();
    merge_sim3_slw_ = Sim3::Identity();
}

Sim3 LoopClosing::Sim3CurrentFromMatched(
    const Sim3& Scw, const std::shared_ptr<KeyFrame>& matched) const {
    // Scw = Scm ∘ Smw  ⇒  Scm = Scw ∘ Smw⁻¹, Smw = Tcw_matched.
    const Sim3 Smw = Sim3::FromSe3(matched->GetPose());
    return Scw.Compose(Smw.Inverse());
}

void LoopClosing::PauseLocalMapping() {
    if (!local_mapping_) {
        return;
    }
    local_mapping_->RequestPause();
    local_mapping_->EmptyQueue();
    while (!local_mapping_->isStopped() && !finish_requested_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void LoopClosing::ResumeLocalMapping() {
    if (local_mapping_) {
        local_mapping_->Release();
    }
}

void LoopClosing::LaunchGlobalBA(Map* active_map, int iterations) {
    if (!active_map) {
        return;
    }
    if (running_gba_.exchange(true)) {
        if (stop_gba_flag_) {
            *stop_gba_flag_ = true;
        }
        return;
    }
    auto stop = std::make_shared<bool>(false);
    stop_gba_flag_ = stop;
    const bool imu = active_map->isImuInitialized();
    const uint64_t epoch = ++gba_epoch_;
    auto run = [this, active_map, iterations, imu, stop, epoch]() {
        if (imu) {
            Optimizer::FullInertialBA(active_map, 7, false, stop.get(), true,
                                      1e2f, 1e6f, epoch);
        } else {
            Optimizer::GlobalBundleAdjustment(active_map, iterations,
                                              stop.get(), epoch);
        }
        if (!*stop && !finish_requested_.load()) {
            PauseLocalMapping();
            PropagateGlobalBA(active_map, epoch);
            ResumeLocalMapping();
        }
        running_gba_.store(false);
    };
    if (scheduler_ != nullptr) {
        gba_future_ = scheduler_->Enqueue(std::move(run));
    } else {
        run();
    }
}

bool LoopClosing::ProcessKeyFrame(const std::shared_ptr<KeyFrame>& keyframe) {
    current_keyframe_ = keyframe;
    std::shared_ptr<KeyFrame> matched;
    Sim3 sim3;
    bool is_merge = false;
    std::vector<std::shared_ptr<MapPoint>> matched_mps;
    if (!NewDetectCommonRegions(&matched, &sim3, &is_merge, &matched_mps) ||
        !matched) {
        return false;
    }
    last_sim3_ = sim3;
    PauseLocalMapping();
    bool ok = false;
    Map* gba_map = map_;
    if (is_merge) {
        const bool inertial =
            current_keyframe_ && current_keyframe_->imu_preintegrated;
        ok = inertial ? MergeLocal2(matched, sim3) : MergeLocal(matched, sim3);
        if (ok && current_keyframe_) {
            gba_map = current_keyframe_->GetMap();
        }
    } else {
        ok = ApplyLoopCorrection(matched, sim3, matched_mps);
    }
    ResumeLocalMapping();
    if (ok) {
        LaunchGlobalBA(gba_map, is_merge ? 5 : 10);
    }
    return ok;
}

bool LoopClosing::NewDetectCommonRegions(std::shared_ptr<KeyFrame>* matched_out,
                                         Sim3* sim3_out, bool* is_merge,
                                         std::vector<std::shared_ptr<MapPoint>>*
                                             matched_mps_out) {
    if (matched_out == nullptr || sim3_out == nullptr || is_merge == nullptr ||
        !current_keyframe_ || !keyframe_database_) {
        return false;
    }
    *is_merge = false;
    if (matched_mps_out != nullptr) {
        matched_mps_out->clear();
    }

    Map* cur_map = current_keyframe_->GetMap();
    if (cur_map != nullptr) {
        if (cur_map->isImuInitialized() && !cur_map->GetInertialBA2()) {
            return false;
        }
        if (cur_map->KeyFramesInMap() < kMinMapKeyFrames) {
            return false;
        }
    }

    bool loop_detected_in_kf = false;
    if (loop_num_coincidences_ > 0 && loop_matched_kf_ &&
        loop_last_current_kf_) {
        const SE3 Tcl = current_keyframe_->GetPose() *
                        loop_last_current_kf_->GetPoseInverse();
        Sim3 gScw = Sim3::FromSe3(Tcl).Compose(loop_sim3_slw_);
        int num_proj = 0;
        std::vector<std::shared_ptr<MapPoint>> matched_mps;
        if (DetectAndRefineSim3FromLastKF(current_keyframe_, loop_matched_kf_,
                                          &gScw, &num_proj, &matched_mps)) {
            loop_detected_in_kf = true;
            ++loop_num_coincidences_;
            loop_last_current_kf_ = current_keyframe_;
            loop_sim3_slw_ = gScw;
            loop_matched_mps_ = matched_mps;
            if (loop_num_coincidences_ >= kCoincidenceThresh) {
                *matched_out = loop_matched_kf_;
                *sim3_out =
                    Sim3CurrentFromMatched(gScw, loop_matched_kf_);
                *is_merge = false;
                if (matched_mps_out != nullptr) {
                    *matched_mps_out = matched_mps;
                }
                ResetLoopState();
                return true;
            }
            loop_num_not_found_ = 0;
        } else {
            ++loop_num_not_found_;
            if (loop_num_not_found_ >= 2) {
                ResetLoopState();
            }
        }
    }

    bool merge_detected_in_kf = false;
    if (merge_num_coincidences_ > 0 && merge_matched_kf_ &&
        merge_last_current_kf_) {
        const SE3 Tcl = current_keyframe_->GetPose() *
                        merge_last_current_kf_->GetPoseInverse();
        Sim3 gScw = Sim3::FromSe3(Tcl).Compose(merge_sim3_slw_);
        int num_proj = 0;
        std::vector<std::shared_ptr<MapPoint>> matched_mps;
        if (DetectAndRefineSim3FromLastKF(current_keyframe_, merge_matched_kf_,
                                          &gScw, &num_proj, &matched_mps)) {
            merge_detected_in_kf = true;
            ++merge_num_coincidences_;
            merge_last_current_kf_ = current_keyframe_;
            merge_sim3_slw_ = gScw;
            if (merge_num_coincidences_ >= kCoincidenceThresh) {
                *matched_out = merge_matched_kf_;
                *sim3_out =
                    Sim3CurrentFromMatched(gScw, merge_matched_kf_);
                *is_merge = true;
                ResetMergeState();
                return true;
            }
            merge_num_not_found_ = 0;
        } else {
            ++merge_num_not_found_;
            if (merge_num_not_found_ >= 2) {
                ResetMergeState();
            }
        }
    }

    if (loop_detected_in_kf || merge_detected_in_kf) {
        return false;
    }

    if (current_keyframe_->id < 10 || !current_keyframe_->HasBoW() ||
        !keyframe_database_) {
        return false;
    }

    std::vector<std::shared_ptr<KeyFrame>> loop_cands;
    std::vector<std::shared_ptr<KeyFrame>> merge_cands;
    keyframe_database_->DetectNBestCandidates(current_keyframe_, &loop_cands,
                                              &merge_cands, 3);

    if (!loop_detected_in_kf && !loop_cands.empty()) {
        Sim3 Scw;
        int coincidences = 0;
        std::shared_ptr<KeyFrame> matched;
        std::vector<std::shared_ptr<MapPoint>> bow_matched_mps;
        const bool detected = DetectCommonRegionsFromBoW(
            loop_cands, &matched, &Scw, &coincidences, &bow_matched_mps);
        if (matched) {
            loop_matched_kf_ = matched;
            loop_last_current_kf_ = current_keyframe_;
            loop_sim3_slw_ = Scw;
            loop_matched_mps_ = bow_matched_mps;
            loop_num_coincidences_ = std::max(coincidences, 1);
            loop_num_not_found_ = 0;
            if (detected) {
                *matched_out = matched;
                *sim3_out = Sim3CurrentFromMatched(Scw, matched);
                *is_merge = false;
                if (matched_mps_out != nullptr) {
                    *matched_mps_out = bow_matched_mps;
                }
                ResetLoopState();
                return true;
            }
        }
    }

    if (!merge_detected_in_kf && !merge_cands.empty()) {
        Sim3 Scw;
        int coincidences = 0;
        std::shared_ptr<KeyFrame> matched;
        const bool detected = DetectCommonRegionsFromBoW(
            merge_cands, &matched, &Scw, &coincidences);
        if (matched) {
            merge_matched_kf_ = matched;
            merge_last_current_kf_ = current_keyframe_;
            merge_sim3_slw_ = Scw;
            merge_num_coincidences_ = std::max(coincidences, 1);
            merge_num_not_found_ = 0;
            if (detected) {
                *matched_out = matched;
                *sim3_out = Sim3CurrentFromMatched(Scw, matched);
                *is_merge = true;
                ResetMergeState();
                return true;
            }
        }
    }
    return false;
}

bool LoopClosing::DetectLoop(
    std::vector<std::shared_ptr<KeyFrame>>* candidates) {
    if (candidates == nullptr || !current_keyframe_ || !keyframe_database_) {
        return false;
    }
    candidates->clear();
    if (finish_requested_.load()) {
        finished_.store(true);
        return false;
    }
    if (current_keyframe_->id < 10 || !current_keyframe_->HasBoW()) {
        return false;
    }
    *candidates = keyframe_database_->DetectLoopCandidates(current_keyframe_,
                                                           min_score_);
    return !candidates->empty();
}

int LoopClosing::FindMatchesByProjection(
    const std::shared_ptr<KeyFrame>& current,
    const std::shared_ptr<KeyFrame>& matched, const Sim3& Scw,
    std::vector<std::shared_ptr<MapPoint>>* map_points,
    std::vector<std::shared_ptr<MapPoint>>* matched_mps) {
    if (!current || !matched || map_points == nullptr ||
        matched_mps == nullptr) {
        return 0;
    }
    auto cov = matched->GetBestCovisibilityKeyFrames(kNumCovisibles);
    cov.push_back(matched);

    std::set<std::shared_ptr<MapPoint>> unique;
    map_points->clear();
    for (const auto& kf : cov) {
        if (!kf || kf->isBad()) {
            continue;
        }
        for (const auto& mp : kf->GetMapPointMatches()) {
            if (!mp || mp->isBad() || unique.count(mp) > 0) {
                continue;
            }
            unique.insert(mp);
            map_points->push_back(mp);
        }
    }

    feature::OrbMatcher matcher(0.9f, true);
    matched_mps->assign(current->GetKeyPoints().size(), nullptr);
    return matcher.SearchByProjection(current, Scw, *map_points, matched_mps, 3.f,
                                      1.5f);
}

bool LoopClosing::DetectAndRefineSim3FromLastKF(
    const std::shared_ptr<KeyFrame>& current,
    const std::shared_ptr<KeyFrame>& matched, Sim3* Scw, int* num_proj_matches,
    std::vector<std::shared_ptr<MapPoint>>* matched_mps) {
    if (!current || !matched || Scw == nullptr || num_proj_matches == nullptr ||
        matched_mps == nullptr) {
        return false;
    }
    std::vector<std::shared_ptr<MapPoint>> map_points;
    *num_proj_matches =
        FindMatchesByProjection(current, matched, *Scw, &map_points, matched_mps);
    if (*num_proj_matches < kRefineProjMatches) {
        return false;
    }

    // Scm = Scw ∘ Twc_matched = Scw ∘ Smw⁻¹... use PoseInverse as Twc.
    const Sim3 Swm = Sim3::FromSe3(matched->GetPoseInverse());
    Sim3 Scm = Scw->Compose(Swm);
    bool fixed = fix_scale_;
    if (current->GetMap() && current->GetMap()->isImuInitialized() &&
        !current->GetMap()->GetInertialBA2()) {
        fixed = false;
    }
    const int num_opt = Optimizer::OptimizeSim3(current, matched, *matched_mps,
                                                &Scm, 10.f, fixed, true);
    if (num_opt <= kRefineOptMatches) {
        return false;
    }

    // Update Scw from refined Scm: Scw = Scm ∘ Smw, Smw = Tcw.
    *Scw = Scm.Compose(Sim3::FromSe3(matched->GetPose()));

    std::vector<std::shared_ptr<MapPoint>> matched_rep;
    const int n_rep =
        FindMatchesByProjection(current, matched, *Scw, &map_points, &matched_rep);
    *num_proj_matches = n_rep;
    if (n_rep < kRefineProjMatchesRep) {
        return false;
    }
    *matched_mps = matched_rep;
    return true;
}

bool LoopClosing::DetectCommonRegionsFromBoW(
    const std::vector<std::shared_ptr<KeyFrame>>& bow_candidates,
    std::shared_ptr<KeyFrame>* matched_out, Sim3* Scw_out,
    int* num_coincidences,
    std::vector<std::shared_ptr<MapPoint>>* matched_mps_out) {
    if (matched_out == nullptr || Scw_out == nullptr ||
        num_coincidences == nullptr || !current_keyframe_) {
        return false;
    }
    *matched_out = nullptr;
    *num_coincidences = 0;
    if (matched_mps_out != nullptr) {
        matched_mps_out->clear();
    }

    const auto connected = current_keyframe_->GetConnectedKeyFrames();
    std::unordered_set<KeyFrame*> connected_set;
    for (const auto& [weak, weight] : connected) {
        if (auto kf = weak.lock()) {
            connected_set.insert(kf.get());
        }
    }

    feature::OrbMatcher matcher_bow(0.9f, true);
    feature::OrbMatcher matcher(0.75f, true);

    std::shared_ptr<KeyFrame> best_matched;
    int best_proj = 0;
    int best_coincidences = 0;
    Sim3 best_Scw = Sim3::Identity();
    std::vector<std::shared_ptr<MapPoint>> best_matched_mps;

    for (const auto& candidate : bow_candidates) {
        if (!candidate || candidate->isBad()) {
            continue;
        }
        auto cov = candidate->GetBestCovisibilityKeyFrames(kNumCovisibles);
        if (cov.empty()) {
            cov.push_back(candidate);
        } else {
            cov.insert(cov.begin(), candidate);
        }

        bool near = false;
        for (const auto& kf : cov) {
            if (kf && connected_set.count(kf.get()) > 0) {
                near = true;
                break;
            }
        }
        if (near) {
            continue;
        }

        std::vector<std::shared_ptr<MapPoint>> vp_matched(
            current_keyframe_->GetKeyPoints().size(), nullptr);
        std::set<std::shared_ptr<MapPoint>> seen;
        int num_bow = 0;
        std::shared_ptr<KeyFrame> most_bow_kf = candidate;
        int most_bow_n = 0;

        for (const auto& kf : cov) {
            if (!kf || kf->isBad()) {
                continue;
            }
            std::vector<std::shared_ptr<MapPoint>> matches12;
            const int n = matcher_bow.SearchByBoW(current_keyframe_, kf, &matches12);
            if (n > most_bow_n) {
                most_bow_n = n;
                most_bow_kf = kf;
            }
            for (size_t k = 0; k < matches12.size(); ++k) {
                const auto& mp = matches12[k];
                if (!mp || mp->isBad() || seen.count(mp) > 0) {
                    continue;
                }
                seen.insert(mp);
                if (k < vp_matched.size()) {
                    vp_matched[k] = mp;
                }
                ++num_bow;
            }
        }
        if (num_bow < kBoWMatches || !most_bow_kf) {
            continue;
        }

        bool fixed = fix_scale_;
        if (current_keyframe_->GetMap() &&
            current_keyframe_->GetMap()->isImuInitialized() &&
            !current_keyframe_->GetMap()->GetInertialBA2()) {
            fixed = false;
        }
        Sim3Solver solver(current_keyframe_, most_bow_kf, vp_matched, fixed);
        solver.SetRansacParameters(0.99, kBoWInliers, 300);
        std::vector<bool> inliers;
        int num_inliers = 0;
        Sim3 Scm = solver.Find(&inliers, &num_inliers);
        if (num_inliers < kBoWInliers) {
            continue;
        }

        // Collect covisible map points of most_bow_kf for projection.
        auto cov2 = most_bow_kf->GetBestCovisibilityKeyFrames(kNumCovisibles);
        cov2.push_back(most_bow_kf);
        std::vector<std::shared_ptr<MapPoint>> vp_map_points;
        std::set<std::shared_ptr<MapPoint>> sp_mps;
        for (const auto& kf : cov2) {
            if (!kf) {
                continue;
            }
            for (const auto& mp : kf->GetMapPointMatches()) {
                if (!mp || mp->isBad() || sp_mps.count(mp) > 0) {
                    continue;
                }
                sp_mps.insert(mp);
                vp_map_points.push_back(mp);
            }
        }

        const Sim3 Smw = Sim3::FromSe3(most_bow_kf->GetPose());
        Sim3 Scw = Scm.Compose(Smw);

        std::vector<std::shared_ptr<MapPoint>> vp_matched_mp(
            current_keyframe_->GetKeyPoints().size(), nullptr);
        const int num_proj = matcher.SearchByProjection(
            current_keyframe_, Scw, vp_map_points, &vp_matched_mp, 8.f, 1.5f);
        if (num_proj < kProjMatches) {
            continue;
        }

        const int num_opt = Optimizer::OptimizeSim3(
            current_keyframe_, most_bow_kf, vp_matched_mp, &Scm, 10.f, fixed,
            true);
        if (num_opt < kSim3Inliers) {
            continue;
        }
        Scw = Scm.Compose(Smw);

        vp_matched_mp.assign(current_keyframe_->GetKeyPoints().size(), nullptr);
        const int num_proj_opt = matcher.SearchByProjection(
            current_keyframe_, Scw, vp_map_points, &vp_matched_mp, 5.f, 1.0f);
        if (num_proj_opt < kProjOptMatches) {
            continue;
        }

        // Coincidence: validate Sim3 on current covisible keyframes.
        int n_valid = 0;
        const auto current_cov =
            current_keyframe_->GetBestCovisibilityKeyFrames(kNumCovisibles);
        for (const auto& kf_j : current_cov) {
            if (!kf_j || kf_j->isBad() || n_valid >= kCoincidenceThresh) {
                break;
            }
            const SE3 Tjc =
                kf_j->GetPose() * current_keyframe_->GetPoseInverse();
            Sim3 Sjw = Sim3::FromSe3(Tjc).Compose(Scw);
            int n_proj_j = 0;
            std::vector<std::shared_ptr<MapPoint>> matched_j;
            std::vector<std::shared_ptr<MapPoint>> mps_j;
            n_proj_j =
                FindMatchesByProjection(kf_j, most_bow_kf, Sjw, &mps_j, &matched_j);
            if (n_proj_j >= kRefineProjMatches) {
                ++n_valid;
            }
        }

        if (num_proj_opt > best_proj) {
            best_proj = num_proj_opt;
            best_coincidences = n_valid;
            best_matched = most_bow_kf;
            best_Scw = Scw;
            best_matched_mps = std::move(vp_matched_mp);
        }
    }

    if (best_proj <= 0 || !best_matched) {
        return false;
    }
    *matched_out = best_matched;
    *Scw_out = best_Scw;
    *num_coincidences = best_coincidences;
    if (matched_mps_out != nullptr) {
        *matched_mps_out = best_matched_mps;
    }
    // Immediate accept when ≥3 covisible KFs validate; else keep state for
    // subsequent keyframes (ORB-SLAM3 DetectCommonRegionsFromBoW).
    return best_coincidences >= kCoincidenceThresh;
}

bool LoopClosing::EstimateSim3(const std::shared_ptr<KeyFrame>& matched,
                               Sim3* sim3_out) {
    if (!matched || !current_keyframe_ || sim3_out == nullptr) {
        return false;
    }
    feature::OrbMatcher matcher(0.75f, true);
    std::vector<std::shared_ptr<MapPoint>> matches12;
    const int nmatches =
        matcher.SearchByBoW(current_keyframe_, matched, &matches12);
    if (nmatches < 20) {
        return false;
    }
    Sim3Solver solver(current_keyframe_, matched, matches12, fix_scale_);
    solver.SetRansacParameters(0.99, 20, 300);
    std::vector<bool> inliers;
    int num_inliers = 0;
    Sim3 sim3 = solver.Find(&inliers, &num_inliers);
    if (num_inliers < 20) {
        return false;
    }
    for (size_t i = 0; i < matches12.size() && i < inliers.size(); ++i) {
        if (!inliers[i]) {
            matches12[i].reset();
        }
    }
    // Guided matching under estimated Sim3 (ORB-SLAM2 ComputeSim3 pattern).
    matcher.SearchBySim3(current_keyframe_, matched, &matches12, sim3, 7.5f);
    const int refined = Optimizer::OptimizeSim3(
        current_keyframe_, matched, matches12, &sim3, 10.f, fix_scale_);
    if (refined < 20) {
        return false;
    }
    *sim3_out = sim3;
    return true;
}

void LoopClosing::SearchAndFuse(
    const std::vector<std::shared_ptr<KeyFrame>>& connected,
    const std::vector<std::shared_ptr<MapPoint>>& map_points) {
    feature::OrbMatcher matcher(0.8f, true);
    for (const auto& keyframe : connected) {
        if (!keyframe || keyframe->isBad()) {
            continue;
        }
        // Scw from corrected KF pose (ORB SearchAndFuse connected-KF overload).
        const Sim3 Scw = Sim3::FromSe3(keyframe->GetPose());
        std::vector<std::shared_ptr<MapPoint>> replace_points(map_points.size(),
                                                              nullptr);
        matcher.Fuse(keyframe, Scw, map_points, 4.f, &replace_points);
        for (size_t i = 0; i < map_points.size(); ++i) {
            if (replace_points[i] && map_points[i] && !map_points[i]->isBad()) {
                // Existing KF point is replaced by the loop/merge candidate.
                replace_points[i]->Replace(map_points[i]);
            }
        }
        if (keyframe->HasDualCameraIndex()) {
            matcher.Fuse(keyframe, map_points, 4.f, true);
        }
    }
}

bool LoopClosing::CorrectLoop(const std::shared_ptr<KeyFrame>& loop_keyframe) {
    Sim3 sim3;
    if (!EstimateSim3(loop_keyframe, &sim3)) {
        return false;
    }
    last_sim3_ = sim3;
    PauseLocalMapping();
    const bool ok = ApplyLoopCorrection(loop_keyframe, sim3);
    ResumeLocalMapping();
    if (ok) {
        LaunchGlobalBA(map_, 10);
    }
    return ok;
}

bool LoopClosing::ApplyLoopCorrection(
    const std::shared_ptr<KeyFrame>& loop_keyframe, const Sim3& sim3,
    const std::vector<std::shared_ptr<MapPoint>>& matched_mps) {
    if (!loop_keyframe || !current_keyframe_ || !map_) {
        return false;
    }

    // sim3 is Scm (current←matched). Corrected Scw = Scm ∘ Smw (ORB mg2oLoopScw).
    const Sim3 Scw = sim3.Compose(Sim3::FromSe3(loop_keyframe->GetPose()));

    current_keyframe_->UpdateConnections();
    std::vector<std::shared_ptr<KeyFrame>> connected =
        current_keyframe_->GetBestCovisibilityKeyFrames(20);
    connected.push_back(current_keyframe_);

    // Propagate corrected Sim3 to connected KFs and align MapPoints (ORB CorrectLoop).
    std::unordered_map<KeyFrame*, Sim3> corrected;
    std::unordered_map<KeyFrame*, Sim3> non_corrected;
    corrected[current_keyframe_.get()] = Scw;
    non_corrected[current_keyframe_.get()] =
        Sim3::FromSe3(current_keyframe_->GetPose());

    {
        SE3 Tcw_corr = SE3Identity();
        Tcw_corr.linear() = Scw.rotation;
        const double inv_s =
            (std::abs(Scw.scale) > 1e-12) ? (1.0 / Scw.scale) : 1.0;
        Tcw_corr.translation() = Scw.translation * inv_s;
        current_keyframe_->SetPose(Tcw_corr);
    }

    const SE3 Twc_se3 = [&]() {
        const Sim3& Sn = non_corrected[current_keyframe_.get()];
        SE3 Twc = SE3Identity();
        Twc.linear() = Sn.rotation.transpose();
        Twc.translation() = -Sn.rotation.transpose() * Sn.translation;
        return Twc;
    }();

    for (const auto& kf : connected) {
        if (!kf || kf->isBad() || kf.get() == current_keyframe_.get()) {
            continue;
        }
        const SE3 Tiw = kf->GetPose();
        const SE3 Tic = Tiw * Twc_se3;
        Sim3 Sic = Sim3::FromSe3(Tic);
        const Sim3 Siw_corr = Sic.Compose(Scw);
        corrected[kf.get()] = Siw_corr;
        non_corrected[kf.get()] = Sim3::FromSe3(Tiw);

        SE3 Tiw_corr = SE3Identity();
        Tiw_corr.linear() = Siw_corr.rotation;
        const double inv_s =
            (std::abs(Siw_corr.scale) > 1e-12) ? (1.0 / Siw_corr.scale) : 1.0;
        Tiw_corr.translation() = Siw_corr.translation * inv_s;
        kf->SetPose(Tiw_corr);
    }

    const bool imu_init = map_->isImuInitialized();
    std::unordered_set<MapPoint*> corrected_mps;
    for (const auto& kf : connected) {
        if (!kf || corrected.count(kf.get()) == 0) {
            continue;
        }
        const Sim3& Siw_corr = corrected[kf.get()];
        const Sim3& Siw = non_corrected[kf.get()];
        const Sim3 Swi_corr = Siw_corr.Inverse();

        for (const auto& mp : kf->GetMapPointMatches()) {
            if (!mp || mp->isBad() || corrected_mps.count(mp.get()) > 0) {
                continue;
            }
            const Vec3 Pw = mp->GetWorldPos();
            mp->SetWorldPos(Swi_corr.Map(Siw.Map(Pw)));
            mp->UpdateNormalAndDepth();
            corrected_mps.insert(mp.get());
        }

        if (imu_init && kf->has_velocity) {
            const Mat33 Rcor =
                Siw_corr.rotation.transpose() * Siw.rotation;
            kf->velocity_world = Rcor * kf->velocity_world;
        }
        kf->UpdateConnections();
    }

    // Fuse already-matched loop MapPoints into current KF (ORB CorrectLoop).
    for (size_t i = 0; i < matched_mps.size(); ++i) {
        const auto& loop_mp = matched_mps[i];
        if (!loop_mp || loop_mp->isBad()) {
            continue;
        }
        auto cur_mp = current_keyframe_->GetMapPoint(static_cast<int>(i));
        if (cur_mp && !cur_mp->isBad()) {
            cur_mp->Replace(loop_mp);
        } else {
            current_keyframe_->AddMapPoint(loop_mp, static_cast<int>(i));
            loop_mp->AddObservation(current_keyframe_, static_cast<int>(i));
            loop_mp->ComputeDistinctiveDescriptors();
        }
    }

    current_keyframe_->AddLoopEdge(loop_keyframe);
    loop_keyframe->AddLoopEdge(current_keyframe_);

    // Project loop-side MapPoints into corrected local window and fuse.
    std::unordered_set<MapPoint*> loop_point_set;
    std::vector<std::shared_ptr<MapPoint>> loop_points;
    auto add_loop_mps = [&](const std::shared_ptr<KeyFrame>& kf) {
        if (!kf) {
            return;
        }
        for (const auto& mp : kf->GetMapPoints()) {
            if (mp && !mp->isBad() && loop_point_set.insert(mp.get()).second) {
                loop_points.push_back(mp);
            }
        }
    };
    add_loop_mps(loop_keyframe);
    for (const auto& neigh : loop_keyframe->GetBestCovisibilityKeyFrames(10)) {
        add_loop_mps(neigh);
    }
    SearchAndFuse(connected, loop_points);

    SE3 loop_relative = SE3Identity();
    loop_relative.linear() = sim3.rotation;
    loop_relative.translation() = sim3.translation;
    std::unordered_map<KeyFrame*, SE3> pose_before;
    for (const auto& [kf, sim_before] : non_corrected) {
        pose_before[kf] = Sim3ToSe3(sim_before);
    }
    if (imu_init) {
        Optimizer::OptimizeEssentialGraph4DoF(map_, loop_keyframe,
                                              current_keyframe_, loop_relative,
                                              &pose_before);
    } else {
        Optimizer::OptimizeEssentialGraph(map_, loop_keyframe, current_keyframe_,
                                          loop_relative, &pose_before);
    }

    if (map_) {
        map_->IncreaseChangeIndex();
    }
    return true;
}

bool LoopClosing::MergeLocal(const std::shared_ptr<KeyFrame>& merge_keyframe,
                             const Sim3& sim3_current_from_merge) {
    if (!merge_keyframe || !current_keyframe_ || !multi_map_) {
        return false;
    }
    Map* merge_map = merge_keyframe->GetMap();
    Map* current_map = current_keyframe_->GetMap();
    if (!merge_map || !current_map || merge_map == current_map) {
        // Same map: treat as loop (caller already paused LocalMapping).
        Sim3 sim3;
        if (!EstimateSim3(merge_keyframe, &sim3)) {
            return false;
        }
        last_sim3_ = sim3;
        return ApplyLoopCorrection(merge_keyframe, sim3);
    }

    // Transform merge-map elements into current map frame.
    const auto merge_keyframes = merge_map->GetAllKeyFrames();
    const auto merge_points = merge_map->GetAllMapPoints();
    for (const auto& kf : merge_keyframes) {
        if (!kf) {
            continue;
        }
        // Tcw_c = S_cm * Tcw_m  (camera of merge expressed in current world via Sim3).
        const SE3 Twc_m = kf->GetPose().inverse();
        const Vec3 twc = sim3_current_from_merge.Map(Twc_m.translation());
        SE3 Twc = SE3Identity();
        Twc.linear() =
            sim3_current_from_merge.rotation * Twc_m.rotation();
        Twc.translation() = twc;
        kf->SetPose(Twc.inverse());
        kf->UpdateMap(current_map);
        current_map->AddKeyFrame(kf);
        merge_map->EraseKeyFrame(kf);
    }
    for (const auto& mp : merge_points) {
        if (!mp || mp->isBad()) {
            continue;
        }
        mp->SetWorldPos(sim3_current_from_merge.Map(mp->GetWorldPos()));
        mp->UpdateMap(current_map);
        current_map->AddMapPoint(mp);
        merge_map->EraseMapPoint(mp);
        mp->UpdateNormalAndDepth();
    }

    current_keyframe_->AddMergeEdge(merge_keyframe);
    merge_keyframe->AddMergeEdge(current_keyframe_);

    if (current_map->isImuInitialized()) {
        Optimizer::MergeInertialBA(current_keyframe_, merge_keyframe,
                                   current_map);
        if (!current_map->GetInertialBA2()) {
            Vec3 bg = Vec3::Zero();
            Vec3 ba = Vec3::Zero();
            Optimizer::InertialOptimization(current_map, &bg, &ba);
            current_map->SetInertialBA1(true);
            current_map->SetInertialBA2(true);
            current_map->SetImuInitialized(true);
        }
    } else {
        Optimizer::WeldingBundleAdjustment(current_keyframe_, merge_keyframe,
                                           current_map);
    }

    std::vector<std::shared_ptr<KeyFrame>> window =
        current_keyframe_->GetBestCovisibilityKeyFrames(15);
    window.push_back(current_keyframe_);
    SearchAndFuse(window, merge_points);

    // Mark merge map bad and switch to current.
    for (const auto& m : multi_map_->GetAllMaps()) {
        if (m.get() == merge_map) {
            multi_map_->SetMapBad(m);
            break;
        }
    }
    multi_map_->RemoveBadMaps();
    multi_map_->ChangeMap(
        std::shared_ptr<Map>(current_map, [](Map*) {}));

    current_map->IncreaseChangeIndex();
    return true;
}

bool LoopClosing::MergeLocal2(const std::shared_ptr<KeyFrame>& merge_keyframe,
                              const Sim3& sim3_current_from_merge) {
    if (!merge_keyframe || !current_keyframe_ || !multi_map_) {
        return false;
    }
    Map* merge_map = merge_keyframe->GetMap();
    Map* current_map = current_keyframe_->GetMap();
    if (!merge_map || !current_map || merge_map == current_map) {
        return MergeLocal(merge_keyframe, sim3_current_from_merge);
    }
    if (stop_gba_flag_) {
        *stop_gba_flag_ = true;
    }

    const Sim3 Scw_merge = sim3_current_from_merge.Compose(
        Sim3::FromSe3(merge_keyframe->GetPose()));
    Sim3 transform_current_to_merge =
        Scw_merge.Inverse().Compose(Sim3::FromSe3(current_keyframe_->GetPose()));
    const bool both_inertial = current_map->isImuInitialized() &&
                               (merge_map->isImuInitialized() ||
                                merge_keyframe->imu_preintegrated);
    if (both_inertial && (transform_current_to_merge.scale < 0.90 ||
                          transform_current_to_merge.scale > 1.1)) {
        return false;
    }
    if (both_inertial && current_map->GetInertialBA1()) {
        Eigen::AngleAxisd angle_axis(transform_current_to_merge.rotation);
        Vec3 phi = angle_axis.angle() * angle_axis.axis();
        phi.x() = 0.0;
        phi.y() = 0.0;
        if (phi.norm() < 1e-12) {
            transform_current_to_merge.rotation = Mat33::Identity();
        } else {
            transform_current_to_merge.rotation =
                Eigen::AngleAxisd(phi.norm(), phi.normalized())
                    .toRotationMatrix();
        }
        transform_current_to_merge.scale = 1.0;
    }

    const bool scale_velocity = std::abs(transform_current_to_merge.scale - 1.0) > 1e-6;
    ApplySim3ToMap(current_map, transform_current_to_merge, scale_velocity);

    if (current_map->isImuInitialized() && !current_map->GetInertialBA2()) {
        Vec3 bg = Vec3::Zero();
        Vec3 ba = Vec3::Zero();
        Optimizer::InertialOptimization(current_map, &bg, &ba);
        current_map->SetInertialBA1(true);
        current_map->SetInertialBA2(true);
        current_map->SetImuInitialized(true);
    }

    const auto merge_keyframes = merge_map->GetAllKeyFrames();
    const auto merge_points = merge_map->GetAllMapPoints();
    for (const auto& kf : merge_keyframes) {
        if (!kf) {
            continue;
        }
        kf->UpdateMap(current_map);
        current_map->AddKeyFrame(kf);
        merge_map->EraseKeyFrame(kf);
    }
    for (const auto& mp : merge_points) {
        if (!mp || mp->isBad()) {
            continue;
        }
        mp->UpdateMap(current_map);
        current_map->AddMapPoint(mp);
        merge_map->EraseMapPoint(mp);
    }

    auto child = merge_keyframe->GetParent();
    auto parent = merge_keyframe;
    merge_keyframe->ChangeParent(current_keyframe_);
    while (child) {
        child->EraseChild(parent);
        auto old_parent = child->GetParent();
        child->ChangeParent(parent);
        parent = child;
        child = old_parent;
    }

    current_keyframe_->AddMergeEdge(merge_keyframe);
    merge_keyframe->AddMergeEdge(current_keyframe_);
    Optimizer::MergeInertialBA(current_keyframe_, merge_keyframe, current_map);

    std::vector<std::shared_ptr<KeyFrame>> window =
        current_keyframe_->GetBestCovisibilityKeyFrames(5);
    window.push_back(current_keyframe_);
    if (window.size() > 6) {
        window.resize(6);
    }
    SearchAndFuse(window, merge_points);

    for (const auto& m : multi_map_->GetAllMaps()) {
        if (m.get() == merge_map) {
            multi_map_->SetMapBad(m);
            break;
        }
    }
    multi_map_->RemoveBadMaps();
    multi_map_->ChangeMap(std::shared_ptr<Map>(current_map, [](Map*) {}));
    current_map->IncreaseChangeIndex();
    return true;
}

void LoopClosing::PropagateGlobalBA(Map* map, uint64_t gba_id) {
    if (!map || gba_id == 0) {
        return;
    }
    std::vector<std::shared_ptr<KeyFrame>> queue = map->keyframe_origins;
    if (queue.empty()) {
        for (const auto& keyframe : map->GetAllKeyFrames()) {
            if (keyframe && keyframe->gba_for_kf == gba_id) {
                queue.push_back(keyframe);
            }
        }
    }
    std::unordered_set<KeyFrame*> seen;
    for (size_t head = 0; head < queue.size(); ++head) {
        const auto keyframe = queue[head];
        if (!keyframe || !seen.insert(keyframe.get()).second) {
            continue;
        }
        for (const auto& child : keyframe->GetChildren()) {
            if (!child || child->isBad()) {
                continue;
            }
            if (child->gba_for_kf != gba_id && keyframe->gba_for_kf == gba_id) {
                const SE3 T_child_parent =
                    child->GetPose() * keyframe->GetPoseInverse();
                child->pose_gba = T_child_parent * keyframe->pose_gba;
                if (child->has_velocity) {
                    const Mat33 correction = child->pose_gba.rotation().transpose() *
                                             child->GetPose().rotation();
                    child->velocity_gba = correction * child->velocity_world;
                }
                child->bias_gba = child->imu_bias;
                child->gba_for_kf = gba_id;
            }
            queue.push_back(child);
        }
        if (keyframe->gba_for_kf != gba_id) {
            continue;
        }
        keyframe->pose_before_gba = keyframe->GetPose();
        if (keyframe->has_velocity) {
            keyframe->velocity_world = keyframe->velocity_gba;
        }
        if (map->isImuInitialized()) {
            keyframe->imu_bias = keyframe->bias_gba;
            if (keyframe->imu_preintegrated) {
                keyframe->imu_preintegrated->SetNewBias(keyframe->imu_bias);
            }
        }
        keyframe->SetPose(keyframe->pose_gba);
    }

    for (const auto& map_point : map->GetAllMapPoints()) {
        if (!map_point || map_point->isBad()) {
            continue;
        }
        if (map_point->gba_for_kf == gba_id) {
            map_point->SetWorldPos(map_point->position_gba);
            map_point->UpdateNormalAndDepth();
            continue;
        }
        const auto reference = map_point->GetReferenceKeyFrame();
        if (!reference || reference->gba_for_kf != gba_id) {
            continue;
        }
        const Vec3 camera_point =
            reference->pose_before_gba * map_point->GetWorldPos();
        map_point->SetWorldPos(reference->GetPoseInverse() * camera_point);
        map_point->UpdateNormalAndDepth();
    }
    map->IncreaseChangeIndex();
}

void LoopClosing::RequestFinish() {
    finish_requested_.store(true);
    if (stop_gba_flag_) {
        *stop_gba_flag_ = true;
    }
    keyframe_queue_.BreakAllWait();
    if (gba_future_.valid()) {
        gba_future_.wait();
    }
    if (worker_future_.valid()) {
        worker_future_.wait();
    }
    finished_.store(true);
    async_.store(false);
}

}  // namespace backend
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
