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

//! Lightweight SE3 pose graph for LO/LIO lidar keyframes (g2o slam3d).
//! Not vision LoopClosing / keyframe DB.

#include "autonomy/localization/atlas/type.hpp"

#include <cstdint>
#include <limits>
#include <unordered_map>
#include <vector>

namespace autonomy::localization::atlas {
namespace backend {

class LidarPoseGraph {
public:
    struct Options {
        double motion_trans_noise = 0.1;
        double motion_rot_noise = 0.05;
        double loop_trans_noise = 0.2;
        double loop_rot_noise = 0.1;
        double robust_delta = 1.0;
        bool use_robust_kernel = true;
    };

    LidarPoseGraph() = default;
    explicit LidarPoseGraph(Options options) : options_(std::move(options)) {}

    void Clear();

    void AddKeyframe(std::uint64_t id, const Mat44_t& T_wb);

    //! Loop constraint: measurement T_from_to ≈ T_from^{-1} * T_to.
    void AddLoop(std::uint64_t id_from, std::uint64_t id_to,
                 const Mat44_t& T_from_to, double score);

    bool Optimize(int iters = 20);

    [[nodiscard]] bool GetPose(std::uint64_t id, Mat44_t* T) const;
    [[nodiscard]] Mat44_t GetPoseOr(std::uint64_t id,
                                    const Mat44_t& fallback) const;

    [[nodiscard]] std::size_t num_keyframes() const { return poses_.size(); }
    [[nodiscard]] std::size_t num_loops() const { return loops_.size(); }

private:
    struct EdgeRec {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        std::uint64_t id_from = 0;
        std::uint64_t id_to = 0;
        Mat44_t T_from_to = Mat44_t::Identity();
        double score = 1.0;
        bool is_loop = false;
    };

    static Eigen::Isometry3d ToIso(const Mat44_t& T);
    static Mat44_t FromIso(const Eigen::Isometry3d& iso);
    Eigen::Matrix<double, 6, 6> MotionInfo() const;
    Eigen::Matrix<double, 6, 6> LoopInfo(double score) const;

    Options options_;
    std::unordered_map<std::uint64_t, Mat44_t> poses_;
    std::vector<EdgeRec> odom_edges_;
    std::vector<EdgeRec> loops_;
    std::uint64_t last_id_ = std::numeric_limits<std::uint64_t>::max();
    bool has_last_ = false;
};

}  // namespace backend
}  // namespace autonomy::localization::atlas
