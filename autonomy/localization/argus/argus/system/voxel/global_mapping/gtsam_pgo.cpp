/*
 * Copyright 2026 The Openbot Authors
 */
#include "argus/system/voxel/global_mapping/gtsam_pgo.hpp"

#include <algorithm>
#include <cmath>

#include "glog/logging.h"

#if defined(ARGUS_HAS_GTSAM) && ARGUS_HAS_GTSAM
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#endif

namespace argus {
namespace vxslam {

#if defined(ARGUS_HAS_GTSAM) && ARGUS_HAS_GTSAM

namespace {

gtsam::Pose3 ToPose3(const ImuState& x) {
    return gtsam::Pose3(gtsam::Rot3(x.R), gtsam::Point3(x.p));
}

using NoisePtr = gtsam::noiseModel::Base::shared_ptr;

void AddBetween(gtsam::NonlinearFactorGraph* graph, int i, int j,
                const Eigen::Matrix3d& R, const Eigen::Vector3d& t,
                const NoisePtr& noise) {
    graph->add(gtsam::BetweenFactor<gtsam::Pose3>(
        i, j, gtsam::Pose3(gtsam::Rot3(R), gtsam::Point3(t)), noise));
}

void AddBetweenStates(gtsam::NonlinearFactorGraph* graph, int i, int j,
                      const ImuState& x1, const ImuState& x2,
                      const NoisePtr& noise) {
    const Eigen::Matrix3d R = x1.R.transpose() * x2.R;
    const Eigen::Vector3d t = x1.R.transpose() * (x2.p - x1.p);
    AddBetween(graph, i, j, R, t, noise);
}

void InjectEdges(gtsam::NonlinearFactorGraph* graph, const PgoEdges& batch,
                 const std::vector<int>& ids,
                 const std::vector<int>& stepsizes,
                 const NoisePtr& default_noise) {
    for (const PgoEdge& edge : batch.edges) {
        std::vector<int> step(2, -1);
        if (!edge.IsAdapt(ids, &step)) {
            continue;
        }
        const int mp0 = stepsizes[static_cast<size_t>(step[0])];
        const int mp1 = stepsizes[static_cast<size_t>(step[1])];
        for (size_t i = 0; i < edge.rots.size(); ++i) {
            NoisePtr noise = default_noise;
            if (i < edge.covs.size()) {
                gtsam::Vector v6(6);
                for (int k = 0; k < 6; ++k) {
                    v6[k] = std::max(1e-9, edge.covs[i][k]);
                }
                noise = gtsam::noiseModel::Diagonal::Variances(v6);
            }
            AddBetween(graph, mp0 + edge.ids1[i], mp1 + edge.ids2[i],
                       edge.rots[i], edge.tras[i], noise);
        }
    }
}

}  // namespace

bool OptimizeIsam2(std::vector<std::vector<ScanPosePtr>*>* multimap_scans,
                   std::vector<std::vector<Keyframe*>*>* multimap_keyframes,
                   const PgoEdges& lp_edges, const PgoEdges& gba_edges1,
                   const PgoEdges& gba_edges2, int cur_session_id,
                   bool inject_gba) {
    if (multimap_scans == nullptr || multimap_scans->empty()) {
        return false;
    }

    std::vector<int> ids;
    lp_edges.Connect(cur_session_id, &ids);
    if (ids.empty()) {
        ids.push_back(cur_session_id);
    }
    // Ensure all sessions that exist are considered if connect is sparse.
    for (int s = 0; s < static_cast<int>(multimap_scans->size()); ++s) {
        if (std::find(ids.begin(), ids.end(), s) == ids.end()) {
            // Keep only sessions that have scans.
            if ((*multimap_scans)[static_cast<size_t>(s)] != nullptr &&
                !(*multimap_scans)[static_cast<size_t>(s)]->empty()) {
                // Do not auto-add disconnected maps unless they are cur.
            }
        }
    }

    std::vector<int> stepsizes;
    stepsizes.push_back(0);
    for (int id : ids) {
        if (id < 0 || id >= static_cast<int>(multimap_scans->size()) ||
            (*multimap_scans)[static_cast<size_t>(id)] == nullptr) {
            stepsizes.push_back(stepsizes.back());
            continue;
        }
        stepsizes.push_back(
            stepsizes.back() +
            static_cast<int>(
                (*multimap_scans)[static_cast<size_t>(id)]->size()));
    }
    const int total = stepsizes.back();
    if (total < 2) {
        return false;
    }

    gtsam::Values initial;
    gtsam::NonlinearFactorGraph graph;

    gtsam::Vector v6_init(6);
    v6_init << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4;
    const NoisePtr odom_noise =
        gtsam::noiseModel::Diagonal::Variances(v6_init);

    for (size_t ii = 0; ii < ids.size(); ++ii) {
        const int sid = ids[ii];
        auto* scans = (*multimap_scans)[static_cast<size_t>(sid)];
        if (scans == nullptr) {
            continue;
        }
        const int bsize = stepsizes[ii];
        for (int j = 0; j < static_cast<int>(scans->size()); ++j) {
            const ScanPosePtr& sp = (*scans)[static_cast<size_t>(j)];
            if (!sp) {
                continue;
            }
            const int gid = bsize + j;
            initial.insert(gid, ToPose3(sp->x));
            if (j > 0) {
                const ScanPosePtr& prev = (*scans)[static_cast<size_t>(j - 1)];
                if (!prev) {
                    continue;
                }
                gtsam::Vector samv6(6);
                for (int k = 0; k < 6; ++k) {
                    samv6[k] = prev->v6[k];
                    if (samv6[k] < 1e-12) {
                        samv6[k] = v6_init[k];
                    }
                }
                const NoisePtr noise =
                    gtsam::noiseModel::Diagonal::Variances(samv6);
                AddBetweenStates(&graph, gid - 1, gid, prev->x, sp->x, noise);
            }
        }
    }

    // Prior on first pose of first connected session.
    {
        gtsam::Vector v6_fixd(6);
        v6_fixd << 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9;
        const NoisePtr fixd =
            gtsam::noiseModel::Diagonal::Variances(v6_fixd);
        auto* scans0 = (*multimap_scans)[static_cast<size_t>(ids[0])];
        if (scans0 && !scans0->empty() && (*scans0)[0]) {
            graph.addPrior(0, ToPose3((*scans0)[0]->x), fixd);
        }
    }

    InjectEdges(&graph, lp_edges, ids, stepsizes, odom_noise);
    if (inject_gba) {
        InjectEdges(&graph, gba_edges1, ids, stepsizes, odom_noise);
        InjectEdges(&graph, gba_edges2, ids, stepsizes, odom_noise);
    }

    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    gtsam::ISAM2 isam(parameters);
    isam.update(graph, initial);
    for (int i = 0; i < 5; ++i) {
        isam.update();
    }
    const gtsam::Values results = isam.calculateEstimate();

    for (size_t ii = 0; ii < ids.size(); ++ii) {
        const int sid = ids[ii];
        auto* scans = (*multimap_scans)[static_cast<size_t>(sid)];
        if (scans == nullptr) {
            continue;
        }
        for (int j = stepsizes[ii]; j < stepsizes[ii + 1]; ++j) {
            const int ord = j - stepsizes[ii];
            if (ord < 0 || ord >= static_cast<int>(scans->size()) ||
                !(*scans)[static_cast<size_t>(ord)]) {
                continue;
            }
            if (!results.exists(j)) {
                continue;
            }
            const gtsam::Pose3 pose = results.at<gtsam::Pose3>(j);
            (*scans)[static_cast<size_t>(ord)]->SetState(
                pose.rotation().matrix(),
                Eigen::Vector3d(pose.translation()));
        }
    }

    if (multimap_keyframes != nullptr) {
        for (size_t ii = 0; ii < ids.size(); ++ii) {
            const int sid = ids[ii];
            if (sid < 0 ||
                sid >= static_cast<int>(multimap_keyframes->size())) {
                continue;
            }
            auto* kfs = (*multimap_keyframes)[static_cast<size_t>(sid)];
            auto* scans = (*multimap_scans)[static_cast<size_t>(sid)];
            if (kfs == nullptr || scans == nullptr) {
                continue;
            }
            for (Keyframe* smp : *kfs) {
                if (smp == nullptr || smp->id < 0 ||
                    smp->id >= static_cast<int>(scans->size()) ||
                    !(*scans)[static_cast<size_t>(smp->id)]) {
                    continue;
                }
                smp->x0 = (*scans)[static_cast<size_t>(smp->id)]->x;
            }
        }
    }

    LOG(INFO) << "OptimizeIsam2 sessions=" << ids.size()
              << " poses=" << total << " factors=" << graph.size()
              << " gba=" << inject_gba;
    return true;
}

#else  // !ARGUS_HAS_GTSAM

bool OptimizeIsam2(std::vector<std::vector<ScanPosePtr>*>*,
                   std::vector<std::vector<Keyframe*>*>*, const PgoEdges&,
                   const PgoEdges&, const PgoEdges&, int, bool) {
    LOG(WARNING) << "OptimizeIsam2: build with ARGUS_USE_GTSAM=ON to enable";
    return false;
}

#endif

}  // namespace vxslam
}  // namespace argus
