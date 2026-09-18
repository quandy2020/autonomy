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

#include "autonomy/localization/atlas/backend/lidar_loop_detector.hpp"

#include "autonomy/localization/atlas/mapping/ivox/ivox.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>

#if __has_include(<pcl/registration/ndt.h>)
#define ATLAS_HAS_PCL_NDT 1
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#else
#define ATLAS_HAS_PCL_NDT 0
#endif

namespace autonomy::localization::atlas {
namespace backend {
namespace {

struct VoxelKey {
    int x = 0;
    int y = 0;
    int z = 0;
    bool operator==(const VoxelKey& o) const {
        return x == o.x && y == o.y && z == o.z;
    }
};

struct VoxelKeyHash {
    std::size_t operator()(const VoxelKey& k) const {
        return (static_cast<std::size_t>(k.x) * 73856093u) ^
               (static_cast<std::size_t>(k.y) * 19349663u) ^
               (static_cast<std::size_t>(k.z) * 83492791u);
    }
};

#if ATLAS_HAS_PCL_NDT
using PclCloud = pcl::PointCloud<pcl::PointXYZ>;

PclCloud::Ptr ToPcl(const std::vector<Vec3_t>& pts) {
    auto cloud = std::make_shared<PclCloud>();
    cloud->reserve(pts.size());
    for (const auto& p : pts) {
        if (!p.allFinite()) {
            continue;
        }
        cloud->push_back(
            pcl::PointXYZ(static_cast<float>(p.x()), static_cast<float>(p.y()),
                          static_cast<float>(p.z())));
    }
    cloud->width = static_cast<std::uint32_t>(cloud->size());
    cloud->height = 1;
    cloud->is_dense = false;
    return cloud;
}

PclCloud::Ptr VoxelDown(const PclCloud::Ptr& in, float leaf) {
    if (!in || in->empty()) {
        return in;
    }
    leaf = std::max(leaf, 0.05f);
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(in);
    vg.setLeafSize(leaf, leaf, leaf);
    auto out = std::make_shared<PclCloud>();
    vg.filter(*out);
    return out;
}
#endif

}  // namespace

void LidarLoopDetector::Clear() {
    keyframes_.clear();
    next_id_ = 0;
    last_keyframe_id_ = std::numeric_limits<std::uint64_t>::max();
}

std::vector<Vec3_t> LidarLoopDetector::Downsample(
    const std::vector<Vec3_t>& pts) const {
    std::vector<Vec3_t> out;
    if (pts.empty()) {
        return out;
    }
    const double leaf = std::max(1e-3, options_.voxel_leaf);
    const double inv = 1.0 / leaf;
    std::unordered_set<VoxelKey, VoxelKeyHash> seen;
    seen.reserve(std::min(pts.size(), options_.max_points * 2));
    out.reserve(std::min(pts.size(), options_.max_points));
    for (const auto& p : pts) {
        if (!p.allFinite()) {
            continue;
        }
        VoxelKey k;
        k.x = static_cast<int>(std::floor(p.x() * inv));
        k.y = static_cast<int>(std::floor(p.y() * inv));
        k.z = static_cast<int>(std::floor(p.z() * inv));
        if (!seen.insert(k).second) {
            continue;
        }
        out.push_back(p);
        if (out.size() >= options_.max_points) {
            break;
        }
    }
    return out;
}

std::uint64_t LidarLoopDetector::AddKeyframe(
    const Mat44_t& T_wb, const std::vector<Vec3_t>& cloud_body) {
    auto down = Downsample(cloud_body);
    if (down.empty()) {
        return std::numeric_limits<std::uint64_t>::max();
    }
    Keyframe kf;
    kf.id = next_id_++;
    kf.T_wb = T_wb;
    kf.cloud_body = std::move(down);
    last_keyframe_id_ = kf.id;
    keyframes_.push_back(std::move(kf));
    while (keyframes_.size() > options_.max_keyframes) {
        keyframes_.pop_front();
    }
    return last_keyframe_id_;
}

std::vector<Vec3_t> LidarLoopDetector::BuildCandidateWorldCloud(
    std::size_t cand_index) const {
    std::vector<Vec3_t> tgt_world;
    if (cand_index >= keyframes_.size()) {
        return tgt_world;
    }
    const int radius = std::max(0, options_.submap_kf_radius);
    const std::size_t begin =
        (cand_index > static_cast<std::size_t>(radius))
            ? (cand_index - static_cast<std::size_t>(radius))
            : 0;
    const std::size_t end = std::min(
        keyframes_.size(),
        cand_index + static_cast<std::size_t>(radius) + 1);

    // Stitch nearby keyframes into world (lightning-lm submap target).
    for (std::size_t i = begin; i < end; ++i) {
        const auto& kf = keyframes_[i];
        const Mat33_t Rw = kf.T_wb.block<3, 3>(0, 0);
        const Vec3_t tw = kf.T_wb.block<3, 1>(0, 3);
        tgt_world.reserve(tgt_world.size() + kf.cloud_body.size());
        for (const auto& p : kf.cloud_body) {
            tgt_world.push_back(Rw * p + tw);
        }
    }
    return tgt_world;
}

bool LidarLoopDetector::AlignNdt(const std::vector<Vec3_t>& src_body,
                                 const std::vector<Vec3_t>& tgt_world,
                                 const Mat44_t& T_init, Mat44_t* T_out,
                                 double* score) const {
#if !ATLAS_HAS_PCL_NDT
    (void)src_body;
    (void)tgt_world;
    (void)T_init;
    (void)T_out;
    (void)score;
    return false;
#else
    if (!T_out || src_body.empty() || tgt_world.empty()) {
        return false;
    }
    auto source = ToPcl(src_body);
    auto target = ToPcl(tgt_world);
    if (!source || !target || source->empty() || target->empty()) {
        return false;
    }

    Eigen::Matrix4f Tw = T_init.cast<float>();
    double last_score = 0.0;
    auto output = std::make_shared<PclCloud>();

    const auto& res_list = options_.ndt_resolutions.empty()
                               ? std::vector<double>{10.0, 5.0, 2.0, 1.0}
                               : options_.ndt_resolutions;

    try {
        for (double r : res_list) {
            if (r <= 0.0) {
                continue;
            }
            pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>
                ndt;
            ndt.setTransformationEpsilon(
                static_cast<float>(options_.ndt_trans_eps));
            ndt.setStepSize(static_cast<float>(options_.ndt_step_size));
            ndt.setMaximumIterations(options_.ndt_max_iters);
            ndt.setResolution(static_cast<float>(r));

            auto rough_tgt =
                VoxelDown(target, static_cast<float>(r * 0.1));
            auto rough_src =
                VoxelDown(source, static_cast<float>(r * 0.1));
            if (!rough_tgt || !rough_src || rough_tgt->empty() ||
                rough_src->empty()) {
                continue;
            }
            ndt.setInputTarget(rough_tgt);
            ndt.setInputSource(rough_src);
            ndt.align(*output, Tw);
            Tw = ndt.getFinalTransformation();
            last_score = ndt.getTransformationProbability();
        }
    } catch (...) {
        return false;
    }

    *T_out = Tw.cast<double>();
    if (score) {
        *score = last_score;
    }
    return last_score > 0.0 || !output->empty();
#endif
}

bool LidarLoopDetector::AlignPointToPlane(
    const std::vector<Vec3_t>& src_body,
    const std::vector<Vec3_t>& tgt_world,
    const Mat44_t& T_init,
    Mat44_t* T_out,
    double* inlier_ratio,
    double* mean_residual) const {
    if (!T_out || src_body.empty() || tgt_world.empty()) {
        return false;
    }

    mapping::IVox::Options ivox_opts;
    ivox_opts.resolution = std::max(0.2, options_.voxel_leaf);
    ivox_opts.max_points_per_voxel = 30;
    ivox_opts.max_voxels = 50000;
    ivox_opts.neighbor_search = 1;
    ivox_opts.min_plane_points = 5;
    mapping::IVox map(ivox_opts);
    map.InsertWorldPoints(tgt_world);

    Mat33_t R = T_init.block<3, 3>(0, 0);
    Vec3_t t = T_init.block<3, 1>(0, 3);
    const double max_d2 =
        options_.max_corr_dist * options_.max_corr_dist;

    int best_inliers = 0;
    double best_mean = std::numeric_limits<double>::infinity();

    for (int iter = 0; iter < options_.icp_iters; ++iter) {
        Mat66_t HtH = Mat66_t::Zero();
        Vec6_t Htr = Vec6_t::Zero();
        int inliers = 0;
        double sum_abs = 0.0;

        for (const auto& pb : src_body) {
            const Vec3_t pw = R * pb + t;
            const auto hit = map.EstimatePlane(pw);
            if (!hit.ok) {
                continue;
            }
            const double e = hit.residual;
            if (e * e > max_d2) {
                continue;
            }
            const Vec3_t Rp = R * pb;
            Vec6_t j;
            j.head<3>() = -hit.normal.cross(Rp);
            j.tail<3>() = hit.normal;
            HtH += j * j.transpose();
            Htr += j * (-e);
            sum_abs += std::abs(e);
            ++inliers;
        }

        if (inliers < 20) {
            break;
        }
        HtH.diagonal().array() += 1e-4;
        const Vec6_t dx = HtH.ldlt().solve(Htr);
        const double ang = dx.head<3>().norm();
        if (ang > 1e-12) {
            R = Eigen::AngleAxisd(ang, dx.head<3>().normalized())
                    .toRotationMatrix() *
                R;
        }
        t += dx.tail<3>();

        best_inliers = inliers;
        best_mean = sum_abs / static_cast<double>(inliers);
        if (dx.norm() < 1e-5) {
            break;
        }
    }

    T_out->setIdentity();
    T_out->block<3, 3>(0, 0) = R;
    T_out->block<3, 1>(0, 3) = t;
    const double ratio =
        static_cast<double>(best_inliers) /
        static_cast<double>(std::max<std::size_t>(1, src_body.size()));
    if (inlier_ratio) {
        *inlier_ratio = ratio;
    }
    if (mean_residual) {
        *mean_residual = best_mean;
    }
    return best_inliers >= 20;
}

bool LidarLoopDetector::Detect(const Mat44_t& T_wb,
                               const std::vector<Vec3_t>& cloud_query,
                               LidarLoopResult* result) {
    if (result) {
        *result = LidarLoopResult{};
    }
    auto query = Downsample(cloud_query);
    if (query.empty() || keyframes_.size() <=
                             static_cast<std::size_t>(
                                 std::max(0, options_.skip_recent_n))) {
        return false;
    }

    const Vec3_t tw = T_wb.block<3, 1>(0, 3);
    const std::size_t n = keyframes_.size();
    const std::size_t skip =
        static_cast<std::size_t>(std::max(0, options_.skip_recent_n));
    const std::size_t end = (n > skip) ? (n - skip) : 0;
    const std::uint64_t query_id = last_keyframe_id_;

    LidarLoopResult best;
    best.query_id = query_id;

    for (std::size_t i = 0; i < end; ++i) {
        const auto& kf = keyframes_[i];
        const Vec3_t dt = kf.T_wb.block<3, 1>(0, 3) - tw;
        if (dt.norm() > options_.candidate_radius_m) {
            continue;
        }

        const std::vector<Vec3_t> tgt_world = BuildCandidateWorldCloud(i);
        if (tgt_world.empty()) {
            continue;
        }

        Mat44_t T_aligned;
        bool accepted = false;
        double ndt_score = 0.0;
        double inlier_ratio = 0.0;
        double mean_res = 0.0;

        if (options_.use_ndt) {
            try {
                if (AlignNdt(query, tgt_world, T_wb, &T_aligned,
                             &ndt_score) &&
                    ndt_score > options_.ndt_score_thresh) {
                    accepted = true;
                }
            } catch (...) {
                accepted = false;
            }
        }

        if (!accepted) {
            if (!AlignPointToPlane(query, tgt_world, T_wb, &T_aligned,
                                   &inlier_ratio, &mean_res)) {
                continue;
            }
            if (inlier_ratio < options_.inlier_ratio_thresh ||
                mean_res > options_.mean_residual_thresh) {
                continue;
            }
            accepted = true;
            ndt_score = 0.0;
        }

        if (!accepted) {
            continue;
        }

        const Mat44_t T_delta = kf.T_wb.inverse() * T_aligned;
        const bool better =
            !best.found ||
            (ndt_score > 0.0 && ndt_score > best.ndt_score) ||
            (ndt_score <= 0.0 && mean_res < best.mean_residual);
        if (better) {
            best.found = true;
            best.query_id = query_id;
            best.candidate_id = kf.id;
            best.T_delta = T_delta;
            best.ndt_score = ndt_score;
            best.inlier_ratio = inlier_ratio;
            best.mean_residual = mean_res;
        }
    }

    if (result) {
        *result = best;
    }
    return best.found;
}

}  // namespace backend
}  // namespace autonomy::localization::atlas
