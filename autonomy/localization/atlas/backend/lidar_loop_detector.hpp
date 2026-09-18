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

//! backend/lidar_loop_detector — lidar loop via PCL multi-res NDT (lightning-lm)
//! with point-plane ICP fallback. No vision LoopClosing merge.

#include "autonomy/localization/atlas/type.hpp"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <vector>

namespace autonomy::localization::atlas {
namespace backend {

struct LidarLoopResult {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    bool found = false;
    std::uint64_t query_id = 0;
    std::uint64_t candidate_id = 0;
    Mat44_t T_delta = Mat44_t::Identity();  // T_cand^{-1} * T_query_aligned
    double ndt_score = 0.0;
    double inlier_ratio = 0.0;
    double mean_residual = 0.0;
};

class LidarLoopDetector {
public:
    struct Options {
        double candidate_radius_m = 15.0;
        int skip_recent_n = 30;
        std::size_t max_keyframes = 500;
        std::size_t max_points = 500;
        double voxel_leaf = 0.5;
        int icp_iters = 8;
        double max_corr_dist = 1.5;
        double inlier_ratio_thresh = 0.35;
        double mean_residual_thresh = 0.25;

        //! Prefer PCL multi-resolution NDT (lightning-lm style).
        bool use_ndt = true;
        std::vector<double> ndt_resolutions = {10.0, 5.0, 2.0, 1.0};
        int ndt_max_iters = 40;
        double ndt_step_size = 0.7;
        double ndt_trans_eps = 0.05;
        //! Higher getTransformationProbability is better (lightning).
        double ndt_score_thresh = 1.0;
        //! Stitch ±radius keyframes as NDT target (0 = single KF cloud).
        int submap_kf_radius = 5;
    };

    struct Keyframe {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        std::uint64_t id = 0;
        Mat44_t T_wb = Mat44_t::Identity();
        std::vector<Vec3_t> cloud_body;
    };

    LidarLoopDetector() = default;
    explicit LidarLoopDetector(Options options)
        : options_(std::move(options)) {}

    //! Returns assigned keyframe id, or max uint64 on empty/failed insert.
    std::uint64_t AddKeyframe(const Mat44_t& T_wb,
                              const std::vector<Vec3_t>& cloud_body);

    //! Spatial candidates + NDT (preferred) / point-to-plane ICP. True on accept.
    [[nodiscard]] bool Detect(const Mat44_t& T_wb,
                              const std::vector<Vec3_t>& cloud_query,
                              LidarLoopResult* result = nullptr);

    //! Compat stub signature (no cloud → always false).
    [[nodiscard]] bool Detect(const Mat44_t& /*T_wb*/) const { return false; }

    void Clear();
    [[nodiscard]] const Options& options() const { return options_; }
    [[nodiscard]] std::size_t num_keyframes() const {
        return keyframes_.size();
    }
    [[nodiscard]] std::uint64_t last_keyframe_id() const {
        return last_keyframe_id_;
    }

private:
    std::vector<Vec3_t> Downsample(const std::vector<Vec3_t>& pts) const;
    bool AlignPointToPlane(const std::vector<Vec3_t>& src_body,
                           const std::vector<Vec3_t>& tgt_world,
                           const Mat44_t& T_init,
                           Mat44_t* T_out,
                           double* inlier_ratio,
                           double* mean_residual) const;

    //! Multi-res NDT: target world cloud, source body cloud, guess = T_wb query.
    bool AlignNdt(const std::vector<Vec3_t>& src_body,
                  const std::vector<Vec3_t>& tgt_world,
                  const Mat44_t& T_init,
                  Mat44_t* T_out,
                  double* score) const;

    std::vector<Vec3_t> BuildCandidateWorldCloud(std::size_t cand_index) const;

    Options options_;
    std::deque<Keyframe> keyframes_;
    std::uint64_t next_id_ = 0;
    std::uint64_t last_keyframe_id_ =
        std::numeric_limits<std::uint64_t>::max();
};

}  // namespace backend
}  // namespace autonomy::localization::atlas
