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
 *
 * Adapted from ORB-SLAM3 ORBmatcher (Tracking subset: projection + Hamming).
 */

/**
 * @file orb_matcher.hpp
 * @brief ORB descriptor matcher: projection search, BoW, triangulation, and fusion
 *        (ORB-SLAM3 ORBmatcher subset).
 *
 * Shared by Tracking, LocalMapping, LoopClosing, etc.; primarily Hamming distance
 * and nearest/next-nearest ratio tests.
 */

#ifndef AUTONOMY_LOCALIZATION_ATLAS_FRONTEND_MATCH_ORB_MATCHER_HPP_
#define AUTONOMY_LOCALIZATION_ATLAS_FRONTEND_MATCH_ORB_MATCHER_HPP_

#include <memory>
#include <set>
#include <utility>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "autonomy/localization/atlas/backend/sim3.hpp"
#include "autonomy/localization/atlas/frontend/tracking/frame.hpp"
#include "autonomy/localization/atlas/map/keyframe.hpp"
#include "autonomy/localization/atlas/map/map_point.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
namespace feature {

/**
 * @class autonomy::localization::atlas::feature::OrbMatcher
 * @brief ORB matching helpers for tracking / mapping / loop closing
 *        (ORB-SLAM3 ORBmatcher subset).
 *
 * Typical uses: motion-model projection (SearchByProjection), reference-keyframe
 * BoW, local-map search, initialization-window matching, keyframe fusion, and
 * Sim3 projection.
 *
 * @note No shared mutable internal state; safe to instantiate per Tracking /
 *       LocalMapping thread.
 */
class OrbMatcher {
public:
    static constexpr int kThLow = 50;        ///< Loose Hamming threshold
    static constexpr int kThHigh = 100;      ///< Strict Hamming threshold
    static constexpr int kHistoLength = 30;  ///< Orientation-consistency histogram bins

    /**
     * @brief Construct the matcher.
     * @param nn_ratio Nearest/next-nearest distance ratio (smaller = stricter).
     * @param check_orientation Whether to reject via rotation-histogram consistency.
     */
    explicit OrbMatcher(float nn_ratio = 0.6f, bool check_orientation = true);

    /**
     * @brief Hamming distance between two ORB descriptors (32 bytes).
     * @param a,b Single-row descriptors.
     * @return Distance (smaller = more similar).
     */
    static int DescriptorDistance(const cv::Mat& a, const cv::Mat& b);

    /**
     * @brief Project last-frame map points into the current frame (TrackWithMotionModel).
     * @param current Current frame (writes map_points).
     * @param last Previous frame.
     * @param th Search radius (pixels; scaled by octave).
     * @param monocular Whether monocular (affects search window).
     * @return Number of new matches.
     */
    int SearchByProjection(tracking::Frame& current,
                           const tracking::Frame& last, float th,
                           bool monocular);

    /**
     * @brief Project visible local map points into the current frame (TrackLocalMap).
     * @param frame Current frame.
     * @param map_points Candidate local map points.
     * @param th Search radius.
     * @param far_points Whether to handle far points separately.
     * @param th_far_points Far-point search radius.
     * @return Match count.
     */
    int SearchByProjection(
        tracking::Frame& frame,
        const std::vector<std::shared_ptr<MapPoint>>& map_points, float th = 3.f,
        bool far_points = false, float th_far_points = 50.f);

    /**
     * @brief Relocalization: project keyframe map points into Frame, skipping found ones.
     * @param current Current frame.
     * @param keyframe Candidate keyframe.
     * @param already_found Already-matched map points.
     * @param th Search radius.
     * @param orb_dist Descriptor distance upper bound.
     * @return Match count.
     */
    int SearchByProjection(
        tracking::Frame& current, const std::shared_ptr<KeyFrame>& keyframe,
        const std::set<std::shared_ptr<MapPoint>>& already_found, float th,
        int orb_dist);

    /**
     * @brief Project map points into a keyframe via Sim3 Scw (loop SearchByProjection).
     * @param keyframe Target keyframe.
     * @param Scw World→camera Sim3.
     * @param map_points Points to project.
     * @param[out] matched Matched map points (index convention of the impl).
     * @param th Search radius.
     * @param ratio_hamming Hamming threshold scale.
     * @return Match count.
     */
    int SearchByProjection(
        const std::shared_ptr<KeyFrame>& keyframe, const backend::Sim3& Scw,
        const std::vector<std::shared_ptr<MapPoint>>& map_points,
        std::vector<std::shared_ptr<MapPoint>>* matched, float th = 3.f,
        float ratio_hamming = 1.0f);

    /**
     * @brief Brute-force keyframe↔frame matching when BoW is not ready
     *        (TrackReferenceKeyFrame).
     * @param keyframe Reference keyframe.
     * @param frame Current frame.
     * @param[out] matches Map-point pointers aligned with frame features (may be null).
     * @return Match count.
     */
    int SearchByBruteForce(
        const std::shared_ptr<KeyFrame>& keyframe, tracking::Frame& frame,
        std::vector<std::shared_ptr<MapPoint>>* matches);

    /**
     * @brief Keyframe↔frame BoW-node matching; falls back to brute force if needed
     *        (ORB SearchByBoW).
     * @param keyframe Keyframe.
     * @param frame Current frame.
     * @param[out] matches Matched map-point list.
     * @return Match count.
     */
    int SearchByBoW(const std::shared_ptr<KeyFrame>& keyframe,
                    tracking::Frame& frame,
                    std::vector<std::shared_ptr<MapPoint>>* matches);

    /**
     * @brief Keyframe↔keyframe BoW matching (loop / Sim3 solve).
     * @param keyframe1,keyframe2 The two keyframes.
     * @param[out] matches12 KF2 map points aligned with KF1 features.
     * @return Match count.
     */
    int SearchByBoW(const std::shared_ptr<KeyFrame>& keyframe1,
                    const std::shared_ptr<KeyFrame>& keyframe2,
                    std::vector<std::shared_ptr<MapPoint>>* matches12);

    /**
     * @brief Project map points into a keyframe and fuse duplicates (LocalMapping).
     * @param keyframe Target keyframe.
     * @param map_points Points to fuse.
     * @param th Search radius.
     * @param right Fisheye right camera (ORB Fuse bRight).
     * @return Number of fusions.
     */
    int Fuse(const std::shared_ptr<KeyFrame>& keyframe,
             const std::vector<std::shared_ptr<MapPoint>>& map_points,
             float th = 3.f, bool right = false);

    /**
     * @brief Loop fusion under corrected Sim3 Scw; collect points that need replace.
     * @param keyframe Target keyframe.
     * @param Scw Corrected Sim3.
     * @param map_points Candidate points.
     * @param th Search radius.
     * @param[out] replace_points Existing map points that should be replaced.
     * @return Number of fusions.
     */
    int Fuse(const std::shared_ptr<KeyFrame>& keyframe, const backend::Sim3& Scw,
             const std::vector<std::shared_ptr<MapPoint>>& map_points, float th,
             std::vector<std::shared_ptr<MapPoint>>* replace_points);

    /**
     * @brief Bidirectional projection matching under Sim3 S12 (ORB SearchBySim3).
     * @param keyframe1,keyframe2 The two keyframes.
     * @param[out] matches12 Match results.
     * @param S12 Sim3 from KF1→KF2.
     * @param th Search radius.
     * @return Match count.
     */
    int SearchBySim3(const std::shared_ptr<KeyFrame>& keyframe1,
                     const std::shared_ptr<KeyFrame>& keyframe2,
                     std::vector<std::shared_ptr<MapPoint>>* matches12,
                     const backend::Sim3& S12, float th);

    /**
     * @brief Search untracked feature pairs for triangulation
     *        (ORB SearchForTriangulation).
     * @param keyframe1,keyframe2 Covisible keyframes.
     * @param[out] matched_pairs Feature pairs (idx1, idx2).
     * @param only_stereo If true, only stereo points.
     * @return Number of matched pairs.
     */
    int SearchForTriangulation(
        const std::shared_ptr<KeyFrame>& keyframe1,
        const std::shared_ptr<KeyFrame>& keyframe2,
        std::vector<std::pair<size_t, size_t>>* matched_pairs,
        bool only_stereo = false);

    /**
     * @brief Monocular initialization window matching + rotation histogram
     *        (ORB SearchForInitialization).
     * @param f1,f2 The two initial frames.
     * @param[in,out] prev_matched Previous-match pixel positions (guided window).
     * @param[out] matches12 f1→f2 indices; negative if unmatched.
     * @param window_size Search-window half-width.
     * @return Match count.
     */
    int SearchForInitialization(tracking::Frame& f1, tracking::Frame& f2,
                                std::vector<cv::Point2f>* prev_matched,
                                std::vector<int>* matches12,
                                int window_size = 100);

private:
    float RadiusByViewingCos(float view_cos) const;
    void ComputeThreeMaxima(std::vector<int>* histo, int length, int* ind1,
                            int* ind2, int* ind3) const;

    float nn_ratio_ = 0.6f;           ///< Nearest-neighbor ratio
    bool check_orientation_ = true;  ///< Whether to check orientation consistency
};

}  // namespace feature
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy

#endif  // AUTONOMY_LOCALIZATION_ATLAS_FRONTEND_MATCH_ORB_MATCHER_HPP_
