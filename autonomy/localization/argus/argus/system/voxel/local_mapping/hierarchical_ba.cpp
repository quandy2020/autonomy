/*
 * Copyright 2026 The Openbot Authors
 */
#include "argus/system/voxel/local_mapping/hierarchical_ba.hpp"

#include <cmath>
#include <unordered_map>
#include <unordered_set>

#include <iostream>

#include "argus/common/rigid_transform.hpp"
#include "argus/system/voxel/local_mapping/lidar_factor.hpp"

namespace argus {
namespace vxslam {

HierarchicalBa::HierarchicalBa(GbaConfig config) : config_(std::move(config)) {}

double HierarchicalBa::Optimize(
    std::vector<ImuState>* poses,
    const std::vector<sensor::PointCloudXYZI>& clouds) {
    if (poses == nullptr || poses->empty() || clouds.size() != poses->size()) {
        return -1.0;
    }
    const int n = static_cast<int>(poses->size());
    gba_voxel_size = config_.voxel_size;
    gba_min_eigen_value = config_.min_eigen_value;
    gba_eigen_value_array = config_.eigen_value_array;
    for (double& v : gba_eigen_value_array) {
        if (v > 1e-9) {
            v = 1.0 / v;
        }
    }

    double last_res = 0.0;
    for (int it = 0; it < config_.total_max_iter; ++it) {
        std::unordered_map<VoxelKey, OctreeGba*> feat_map;
        for (int i = 0; i < n; ++i) {
            OctreeGba::CutVoxel(&feat_map, (*poses)[static_cast<size_t>(i)],
                                clouds[static_cast<size_t>(i)], i, n);
        }
        LidarFactor voxhess(n);
        OctreeGbaMultiRecut(&feat_map, &voxhess, config_.thread_num);
        if (voxhess.plvec_voxels.size() < 5) {
            std::cerr << "HierarchicalBa: too few voxels\n";
            return last_res;
        }
        LidarBaOptimizer opt;
        std::vector<double> resis;
        Eigen::MatrixXd hess;
        opt.DampingIter(*poses, voxhess, &hess, resis, /*max_iter=*/3,
                        /*is_display=*/false);
        if (!resis.empty()) {
            last_res = resis.back();
        }
    }
    return last_res;
}

void HierarchicalBa::HbaAddEdge(std::vector<ImuState>* xs,
                                const std::vector<Keyframe*>& smps,
                                PgoEdges* gba_edges,
                                const std::vector<int>& maps, int max_iter,
                                int thread_num) {
    if (xs == nullptr || gba_edges == nullptr || xs->size() != smps.size()) {
        return;
    }
    std::unordered_set<int> map_set(maps.begin(), maps.end());
    std::vector<Keyframe*> smps_f;
    std::vector<ImuState> xs_f;
    int last_mp = -1;
    bool is_cnct = false;
    for (size_t i = 0; i < smps.size(); ++i) {
        Keyframe* smp = smps[i];
        if (smp == nullptr) {
            continue;
        }
        if (smp->mp != last_mp) {
            is_cnct = map_set.count(smp->mp) > 0;
            last_mp = smp->mp;
        }
        if (is_cnct) {
            smps_f.push_back(smp);
            xs_f.push_back((*xs)[i]);
        }
    }
    const int wdsize = static_cast<int>(smps_f.size());
    if (wdsize < 2) {
        return;
    }

    const std::vector<double> eigen_orig = gba_eigen_value_array;
    const double min_eig_orig = gba_min_eigen_value;
    const double voxel_orig = gba_voxel_size;

    gba_voxel_size = config_.voxel_size;
    gba_min_eigen_value = config_.min_eigen_value;
    gba_eigen_value_array = config_.eigen_value_array;
    for (double& v : gba_eigen_value_array) {
        if (v > 1e-9) {
            v = 1.0 / v;
        }
    }

    Eigen::MatrixXd hess;
    int converge_flag = 0;
    double converge_thre = 0.05;
    const int up = 4;
    const int thd = thread_num > 0 ? thread_num : config_.thread_num;

    for (int iter = 0; iter < max_iter; ++iter) {
        if (converge_flag == 1 || iter == max_iter - 1) {
            gba_voxel_size = voxel_size;
            gba_eigen_value_array = plane_eigen_value_thre;
            gba_min_eigen_value = min_eigen_value;
        }
        std::unordered_map<VoxelKey, OctreeGba*> oct_map;
        for (int i = 0; i < wdsize; ++i) {
            if (smps_f[static_cast<size_t>(i)]->plptr == nullptr) {
                continue;
            }
            OctreeGba::CutVoxel(&oct_map, xs_f[static_cast<size_t>(i)],
                                *smps_f[static_cast<size_t>(i)]->plptr, i,
                                wdsize);
        }
        LidarFactor voxhess(wdsize);
        OctreeGbaMultiRecut(&oct_map, &voxhess, thd);
        LidarBaOptimizer opt;
        opt.thd_num = thd;
        std::vector<double> resis;
        const bool is_converge =
            opt.DampingIter(xs_f, voxhess, &hess, resis, up, false);
        if (resis.size() >= 2 &&
            ((std::fabs(resis[0] - resis[1]) / std::max(resis[0], 1e-12) <
                  converge_thre &&
              is_converge) ||
             (iter == max_iter - 2 && converge_flag == 0))) {
            converge_thre = 0.01;
            if (converge_flag == 0) {
                converge_flag = 1;
            } else {
                break;
            }
        }
    }

    gba_eigen_value_array = eigen_orig;
    gba_min_eigen_value = min_eig_orig;
    gba_voxel_size = voxel_orig;

    if (hess.rows() < 6 * wdsize || hess.cols() < 6 * wdsize) {
        return;
    }
    for (int i = 0; i < wdsize - 1; ++i) {
        for (int j = i + 1; j < wdsize; ++j) {
            bool is_add = true;
            Eigen::Matrix<double, 6, 1> v6;
            for (int k = 0; k < 6; ++k) {
                const double hc =
                    std::fabs(hess(6 * i + k, 6 * j + k));
                if (hc < 1e-6) {
                    is_add = false;
                    break;
                }
                v6[k] = 1.0 / hc;
            }
            if (!is_add) {
                continue;
            }
            Keyframe& s1 = *smps_f[static_cast<size_t>(i)];
            Keyframe& s2 = *smps_f[static_cast<size_t>(j)];
            const Eigen::Vector3d tra =
                xs_f[static_cast<size_t>(i)].R.transpose() *
                (xs_f[static_cast<size_t>(j)].p -
                 xs_f[static_cast<size_t>(i)].p);
            const Eigen::Matrix3d rot =
                xs_f[static_cast<size_t>(i)].R.transpose() *
                xs_f[static_cast<size_t>(j)].R;
            gba_edges->Push(s1.mp, s2.mp, s1.id, s2.id, rot, tra, v6);
        }
    }

    // Write optimized poses back into filtered slots of *xs via identity of
    // smp pointers — map by original index.
    size_t fi = 0;
    last_mp = -1;
    is_cnct = false;
    for (size_t i = 0; i < smps.size() && fi < xs_f.size(); ++i) {
        Keyframe* smp = smps[i];
        if (smp == nullptr) {
            continue;
        }
        if (smp->mp != last_mp) {
            is_cnct = map_set.count(smp->mp) > 0;
            last_mp = smp->mp;
        }
        if (is_cnct) {
            (*xs)[i] = xs_f[fi++];
        }
    }
}

void HierarchicalBa::TopDownApplyEdges(
    backend::PoseGraph* graph, const PgoEdges& gba_edges1,
    const PgoEdges& gba_edges2, const std::vector<int>& ids,
    const std::vector<int>& stepsizes) {
    if (graph == nullptr || ids.empty() ||
        stepsizes.size() != ids.size() + 1) {
        return;
    }
    auto apply = [&](const PgoEdges& edges) {
        for (const PgoEdge& edge : edges.edges) {
            std::vector<int> step(2, -1);
            if (!edge.IsAdapt(ids, &step)) {
                continue;
            }
            const int mp0 = stepsizes[static_cast<size_t>(step[0])];
            const int mp1 = stepsizes[static_cast<size_t>(step[1])];
            for (size_t i = 0; i < edge.rots.size(); ++i) {
                backend::LoopFactor f;
                f.from_id = mp0 + edge.ids1[i];
                f.to_id = mp1 + edge.ids2[i];
                f.T_i_j = transform::Rigid3d(
                    edge.tras[i], Eigen::Quaterniond(edge.rots[i]));
                f.information =
                    1.0 / std::max(edge.covs[i].mean(), 1e-9);
                graph->AddLoopFactor(f);
            }
        }
    };
    apply(gba_edges1);
    apply(gba_edges2);
}

}  // namespace vxslam
}  // namespace argus
