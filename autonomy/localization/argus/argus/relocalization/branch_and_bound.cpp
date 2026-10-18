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

#include "argus/relocalization/branch_and_bound.hpp"

#include <cmath>
#include <queue>
#include <utility>
#include <vector>

#include "argus/common/math/so3.hpp"
#include "argus/common/math/transform/transform.hpp"

namespace argus {
namespace relocalization {

BranchAndBound::BranchAndBound(PriorOccupancyMap::ConstSharedPtr map,
                               BranchAndBoundConfig config)
    : map_(std::move(map)), config_(std::move(config)) {}

transform::Rigid3d BranchAndBound::MakePose(double x, double y, double z,
                                            double yaw) const {
    return MakePoseFull(x, y, z, 0.0, 0.0, yaw);
}

transform::Rigid3d BranchAndBound::MakePoseFull(double x, double y, double z,
                                                double roll, double pitch,
                                                double yaw) const {
    return transform::FromXyzRpy(x, y, z, roll, pitch, yaw);
}

double BranchAndBound::ScoreAt(const sensor::PointCloudXYZI& cloud_b, double x,
                            double y, double z, double yaw) const {
    return ScoreAtPose(cloud_b, MakePose(x, y, z, yaw));
}

double BranchAndBound::ScoreAtPose(const sensor::PointCloudXYZI& cloud_b,
                                const transform::Rigid3d& T) const {
    if (!map_) {
        return 0.0;
    }
    return map_->Score(cloud_b, T);
}

transform::Rigid3d BranchAndBound::RefineRollPitch(
    const sensor::PointCloudXYZI& cloud_b, const transform::Rigid3d& T_seed,
    double* best_score) const {
    if (config_.roll_pitch_refine_rad <= 0.0 || config_.roll_pitch_steps <= 0) {
        return T_seed;
    }
    const Eigen::Vector3d t = T_seed.translation();
    const double yaw0 = transform::GetYaw(T_seed);

    transform::Rigid3d best_T = T_seed;
    double best =
        best_score != nullptr ? *best_score : ScoreAtPose(cloud_b, T_seed);
    const int n = std::max(1, config_.roll_pitch_steps);
    const double half = config_.roll_pitch_refine_rad;
    for (int ir = 0; ir < n; ++ir) {
        const double roll =
            (n == 1) ? 0.0 : (-half + 2.0 * half * ir / (n - 1));
        for (int ip = 0; ip < n; ++ip) {
            const double pitch =
                (n == 1) ? 0.0 : (-half + 2.0 * half * ip / (n - 1));
            const transform::Rigid3d T =
                MakePoseFull(t.x(), t.y(), t.z(), roll, pitch, yaw0);
            const double s = ScoreAtPose(cloud_b, T);
            if (s > best) {
                best = s;
                best_T = T;
            }
        }
    }
    if (best_score != nullptr) {
        *best_score = best;
    }
    return best_T;
}

transform::Rigid3d BranchAndBound::RefineSE3(
    const sensor::PointCloudXYZI& cloud_b, const transform::Rigid3d& T_seed,
    double* best_score) const {
    // Seed with roll/pitch grid, then coordinate-descent on 6 DoF.
    transform::Rigid3d best_T = RefineRollPitch(cloud_b, T_seed, best_score);
    double best =
        best_score != nullptr ? *best_score : ScoreAtPose(cloud_b, best_T);

    const int n = std::max(1, config_.roll_pitch_steps);
    const double ht = config_.se3_trans_refine_m;
    const double hy = config_.se3_yaw_refine_rad;
    const double hr = config_.roll_pitch_refine_rad;
    const int passes = std::max(1, config_.se3_refine_passes);

    auto sample1d = [n](double half, int i) {
        return (n == 1) ? 0.0 : (-half + 2.0 * half * i / (n - 1));
    };

    for (int pass = 0; pass < passes; ++pass) {
        Eigen::Vector3d t = best_T.translation();
        const Eigen::Vector3d rpy = math::RotationMatrixToRollPitchYaw(
            best_T.rotation().toRotationMatrix());
        double roll = rpy.x();
        double pitch = rpy.y();
        double yaw = rpy.z();

        // Optimize each DoF independently (coordinate descent).
        const double* axes_half[6] = {&ht, &ht, &ht, &hr, &hr, &hy};
        double* axes_val[6] = {&t.x(), &t.y(), &t.z(), &roll, &pitch, &yaw};
        for (int a = 0; a < 6; ++a) {
            double local_best = best;
            double best_v = *axes_val[a];
            for (int i = 0; i < n; ++i) {
                const double v = *axes_val[a] + sample1d(*axes_half[a], i);
                double cand[6] = {t.x(), t.y(), t.z(), roll, pitch, yaw};
                cand[a] = v;
                const transform::Rigid3d T = MakePoseFull(
                    cand[0], cand[1], cand[2], cand[3], cand[4], cand[5]);
                const double s = ScoreAtPose(cloud_b, T);
                if (s > local_best) {
                    local_best = s;
                    best_v = v;
                    best_T = T;
                }
            }
            *axes_val[a] = best_v;
            best = local_best;
            t = best_T.translation();
            const Eigen::Vector3d rpy2 = math::RotationMatrixToRollPitchYaw(
                best_T.rotation().toRotationMatrix());
            roll = rpy2.x();
            pitch = rpy2.y();
            yaw = rpy2.z();
        }
    }

    if (best_score != nullptr) {
        *best_score = best;
    }
    return best_T;
}

bool BranchAndBound::SeedSearchNodes(const RelocRequest& request,
                                      std::atomic<bool>* cancel,
                                      std::vector<Node>* seeds,
                                      RelocResult* out) {
    const bool search_z = (config_.search_space == BnBSearchSpace::kSE3Yaw ||
                           config_.search_space == BnBSearchSpace::kSE3);

    Eigen::Vector3d c0 = 0.5 * (map_->min_bound() + map_->max_bound());
    double yaw0 = 0.0;
    if (request.has_initial_guess) {
        c0 = request.initial_guess.translation();
        yaw0 = transform::GetYaw(request.initial_guess);
    }

    const double xy0 = config_.resolution;
    const double z0 = search_z ? config_.z_resolution : 0.0;
    const double yaw_step0 = config_.yaw_step_rad;
    const int R = std::max(1, config_.search_radius_steps);
    const int Rz = search_z ? std::max(0, config_.search_radius_z_steps) : 0;

    seeds->clear();
    for (int ix = -R; ix <= R; ++ix) {
        for (int iy = -R; iy <= R; ++iy) {
            for (int iz = -Rz; iz <= Rz; ++iz) {
                for (int iyaw = -R; iyaw <= R; ++iyaw) {
                    if (cancel != nullptr && cancel->load()) {
                        out->cancelled = true;
                        return false;
                    }
                    Node n;
                    n.x = c0.x() + ix * xy0;
                    n.y = c0.y() + iy * xy0;
                    n.z = search_z ? (c0.z() + iz * z0) : c0.z();
                    n.yaw = yaw0 + iyaw * yaw_step0;
                    n.half_xy = 0.5 * xy0;
                    n.half_z = search_z ? 0.5 * z0 : 0.0;
                    n.half_yaw = 0.5 * yaw_step0;
                    n.depth = 0;
                    n.score = ScoreAt(request.query_cloud, n.x, n.y, n.z, n.yaw);
                    seeds->push_back(n);
                }
            }
        }
    }
    return true;
}

void BranchAndBound::ExpandChildren(const Node& cur, bool search_z,
                                    const sensor::PointCloudXYZI& cloud,
                                    std::vector<Node>* children) const {
    children->clear();
    if (cur.depth >= config_.max_depth) {
        return;
    }
    const double hx = 0.5 * cur.half_xy;
    const double hz = search_z ? 0.5 * cur.half_z : 0.0;
    const double hyaw = 0.5 * cur.half_yaw;
    const int sz_opts[2] = {-1, 1};
    const int n_sz = search_z ? 2 : 1;
    for (int sx : {-1, 1}) {
        for (int sy : {-1, 1}) {
            for (int isz = 0; isz < n_sz; ++isz) {
                const int sz = search_z ? sz_opts[isz] : 0;
                for (int syaw : {-1, 1}) {
                    Node ch = cur;
                    ch.x = cur.x + sx * hx;
                    ch.y = cur.y + sy * hx;
                    ch.z = search_z ? (cur.z + sz * hz) : cur.z;
                    ch.yaw = cur.yaw + syaw * hyaw;
                    ch.half_xy = hx;
                    ch.half_z = hz;
                    ch.half_yaw = hyaw;
                    ch.depth = cur.depth + 1;
                    ch.score = ScoreAt(cloud, ch.x, ch.y, ch.z, ch.yaw);
                    if (ch.score + 1e-9 < config_.score_threshold * 0.5 &&
                        ch.depth > config_.max_depth / 2) {
                        continue;
                    }
                    children->push_back(ch);
                }
            }
        }
    }
}

RelocResult BranchAndBound::Search(const RelocRequest& request,
                                   std::atomic<bool>* cancel) {
    RelocResult out;
    out.request_id = request.id;
    if (!map_ || request.query_cloud.empty()) {
        return out;
    }

    const bool search_z = (config_.search_space == BnBSearchSpace::kSE3Yaw ||
                           config_.search_space == BnBSearchSpace::kSE3);

    struct Cmp {
        bool operator()(const Node& a, const Node& b) const {
            return a.score < b.score;  // max-heap
        }
    };
    std::priority_queue<Node, std::vector<Node>, Cmp> open;

    std::vector<Node> seeds;
    if (!SeedSearchNodes(request, cancel, &seeds, &out)) {
        return out;
    }
    for (const Node& n : seeds) {
        open.push(n);
    }

    Node best;
    best.score = -1.0;
    int expansions = 0;
    std::vector<Node> children;

    while (!open.empty() && expansions < config_.max_expansions) {
        if (cancel != nullptr && cancel->load()) {
            out.cancelled = true;
            break;
        }
        Node cur = open.top();
        open.pop();
        ++expansions;

        if (cur.score > best.score) {
            best = cur;
        }
        ExpandChildren(cur, search_z, request.query_cloud, &children);
        for (const Node& ch : children) {
            open.push(ch);
        }
    }

    if (best.score < 0.0) {
        return out;
    }

    transform::Rigid3d T = MakePose(best.x, best.y, best.z, best.yaw);
    double score = best.score;
    if (config_.search_space == BnBSearchSpace::kSE3) {
        T = RefineSE3(request.query_cloud, T, &score);
    } else if (config_.search_space == BnBSearchSpace::kSE3Yaw) {
        T = RefineRollPitch(request.query_cloud, T, &score);
    }

    out.best.T_w_b = T;
    out.best.score = score;
    if (score >= config_.score_threshold) {
        out.success = true;
    }
    return out;
}

}  // namespace relocalization
}  // namespace argus
