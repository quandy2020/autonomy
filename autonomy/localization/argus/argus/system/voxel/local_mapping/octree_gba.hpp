/*
 * Copyright 2026 The Openbot Authors
 * Voxel-SLAM OctreeGBA port (no ROS / PCL publishers).
 */
#ifndef ARGUS_VXSLAM_OCTREE_GBA_HPP_
#define ARGUS_VXSLAM_OCTREE_GBA_HPP_

#include <cstdlib>
#include <thread>
#include <unordered_map>
#include <vector>

#include <Eigen/Eigenvalues>

#include "argus/sensor/point_types.hpp"
#include "argus/system/voxel/odometry/imu_state.hpp"
#include "argus/system/voxel/local_mapping/lidar_factor.hpp"
#include "argus/mapping/point_cluster.hpp"
#include "argus/mapping/voxel_key.hpp"

namespace argus {
namespace vxslam {

inline std::vector<double> gba_eigen_value_array;
inline double gba_min_eigen_value = 0.01;
inline double gba_voxel_size = 1.0;

/** @brief Hierarchical voxel plane builder for global BA. */
class OctreeGba {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::vector<VectorList<3>> locals;
    std::vector<VectorList<3>> worlds;
    PointCluster pcr_add;
    int layer = 0;
    int octo_state = 0;
    int wdsize = 0;
    OctreeGba* leaves[8] = {};
    double voxel_center[3] = {};
    float quater_length = 0.f;
    bool is_plane = false;
    float ref = 0.f;

    OctreeGba(int layer_in, int wd) : layer(layer_in), wdsize(wd) {
        locals.resize(static_cast<size_t>(wdsize));
        worlds.resize(static_cast<size_t>(wdsize));
        ref = 255.0f * static_cast<float>(std::rand()) /
              (static_cast<float>(RAND_MAX) + 1.0f);
    }

    void Reset(int layer_in, int wd) {
        layer = layer_in;
        wdsize = wd;
        locals.clear();
        worlds.clear();
        locals.resize(static_cast<size_t>(wdsize));
        worlds.resize(static_cast<size_t>(wdsize));
        pcr_add.Clear();
        octo_state = 0;
        is_plane = false;
        for (int i = 0; i < 8; ++i) {
            leaves[i] = nullptr;
        }
    }

    bool PlaneJudge(const Eigen::Vector3d& eig_values) const {
        if (layer >= static_cast<int>(gba_eigen_value_array.size())) {
            return eig_values[0] < gba_min_eigen_value;
        }
        return eig_values[0] < gba_min_eigen_value &&
               (eig_values[0] / eig_values[2]) <
                   gba_eigen_value_array[static_cast<size_t>(layer)];
    }

    void Push(int ord, Eigen::Vector3d local, Eigen::Vector3d world) {
        locals[static_cast<size_t>(ord)].push_back(local);
        worlds[static_cast<size_t>(ord)].push_back(world);
        pcr_add.Push(world);
    }

    void Subdivide(std::vector<OctreeGba*>* oct_buf) {
        for (int i = 0; i < wdsize; ++i) {
            for (size_t j = 0; j < locals[static_cast<size_t>(i)].size(); ++j) {
                Eigen::Vector3d& pl = locals[static_cast<size_t>(i)][j];
                Eigen::Vector3d& pw = worlds[static_cast<size_t>(i)][j];
                int xyz[3] = {0, 0, 0};
                for (int k = 0; k < 3; ++k) {
                    if (pw[k] > voxel_center[k]) {
                        xyz[k] = 1;
                    }
                }
                const int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
                if (leaves[leafnum] == nullptr) {
                    if (oct_buf != nullptr && !oct_buf->empty()) {
                        leaves[leafnum] = oct_buf->back();
                        leaves[leafnum]->Reset(layer + 1, wdsize);
                        oct_buf->pop_back();
                    } else {
                        leaves[leafnum] = new OctreeGba(layer + 1, wdsize);
                    }
                    leaves[leafnum]->voxel_center[0] =
                        voxel_center[0] + (2 * xyz[0] - 1) * quater_length;
                    leaves[leafnum]->voxel_center[1] =
                        voxel_center[1] + (2 * xyz[1] - 1) * quater_length;
                    leaves[leafnum]->voxel_center[2] =
                        voxel_center[2] + (2 * xyz[2] - 1) * quater_length;
                    leaves[leafnum]->quater_length = quater_length / 2.f;
                }
                leaves[leafnum]->Push(i, pl, pw);
            }
        }
    }

    void Recut(LidarFactor* vox_opt, std::vector<OctreeGba*>* oct_buf) {
        if (vox_opt == nullptr || pcr_add.N <= 10) {
            return;
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(pcr_add.cov());
        Eigen::Vector3d eig_value = saes.eigenvalues();
        Eigen::Matrix3d eig_vector = saes.eigenvectors();
        is_plane = PlaneJudge(eig_value);
        if (is_plane) {
            if (pcr_add.N < 10) {
                return;
            }
            int exi = 0;
            for (int i = 0; i < wdsize; ++i) {
                if (!locals[static_cast<size_t>(i)].empty()) {
                    ++exi;
                }
            }
            if (exi <= 1) {
                return;
            }
            if (eig_value[0] / eig_value[1] > 0.12) {
                return;
            }
            const double coe = 1.0;
            std::vector<PointCluster> pcrs(static_cast<size_t>(wdsize));
            for (int i = 0; i < wdsize; ++i) {
                for (Eigen::Vector3d& v : locals[static_cast<size_t>(i)]) {
                    pcrs[static_cast<size_t>(i)].Push(v);
                }
            }
            PointCluster pcr_fix;
            vox_opt->push_voxel(pcrs, pcr_fix, coe, eig_value, eig_vector,
                                pcr_add);
            return;
        }
        if (layer >= max_layer) {
            return;
        }
        Subdivide(oct_buf);
        octo_state = 1;
        for (int i = 0; i < 8; ++i) {
            if (leaves[i] != nullptr) {
                leaves[i]->Recut(vox_opt, oct_buf);
            }
        }
    }

    void CollectLeaves(std::vector<OctreeGba*>* oct_buf) {
        if (octo_state != 1 || oct_buf == nullptr) {
            return;
        }
        for (int i = 0; i < 8; ++i) {
            if (leaves[i] != nullptr) {
                leaves[i]->CollectLeaves(oct_buf);
                oct_buf->push_back(leaves[i]);
                leaves[i] = nullptr;
            }
        }
    }

    static void CutVoxel(std::unordered_map<VoxelKey, OctreeGba*>* feat_map,
                         const ImuState& xc,
                         const sensor::PointCloudXYZI& cloud, int win_count,
                         int wdsize) {
        if (feat_map == nullptr) {
            return;
        }
        for (const sensor::PointType& ap : cloud.points) {
            Eigen::Vector3d local(ap.x, ap.y, ap.z);
            Eigen::Vector3d world = xc.R * local + xc.p;
            float loc[3];
            for (int j = 0; j < 3; ++j) {
                loc[j] = static_cast<float>(world[j] / gba_voxel_size);
                if (loc[j] < 0) {
                    loc[j] -= 1.f;
                }
            }
            const VoxelKey position(static_cast<int64_t>(loc[0]),
                                    static_cast<int64_t>(loc[1]),
                                    static_cast<int64_t>(loc[2]));
            auto iter = feat_map->find(position);
            if (iter != feat_map->end()) {
                iter->second->Push(win_count, local, world);
            } else {
                auto* ot = new OctreeGba(0, wdsize);
                ot->Push(win_count, local, world);
                ot->voxel_center[0] =
                    (0.5 + static_cast<double>(position.x)) * gba_voxel_size;
                ot->voxel_center[1] =
                    (0.5 + static_cast<double>(position.y)) * gba_voxel_size;
                ot->voxel_center[2] =
                    (0.5 + static_cast<double>(position.z)) * gba_voxel_size;
                ot->quater_length =
                    static_cast<float>(gba_voxel_size / 4.0);
                (*feat_map)[position] = ot;
            }
        }
    }
};

inline void OctreeGbaMultiRecut(
    std::unordered_map<VoxelKey, OctreeGba*>* feat_map, LidarFactor* voxhess,
    int thd_num) {
    if (feat_map == nullptr || voxhess == nullptr) {
        return;
    }
    if (thd_num < 1) {
        thd_num = 1;
    }
    std::vector<std::vector<OctreeGba*>> octss(static_cast<size_t>(thd_num));
    std::vector<LidarFactor> vec_voxhess(static_cast<size_t>(thd_num),
                                         *voxhess);
    const int g_size = static_cast<int>(feat_map->size());
    if (g_size < thd_num) {
        thd_num = 1;
    }
    const double part = 1.0 * g_size / std::max(thd_num, 1);
    int cnt = 0;
    for (auto& kv : *feat_map) {
        octss[static_cast<size_t>(cnt)].push_back(kv.second);
        if (static_cast<double>(octss[static_cast<size_t>(cnt)].size()) >=
                part &&
            cnt < thd_num - 1) {
            ++cnt;
        }
    }

    auto recut_func = [](std::vector<OctreeGba*>* octs, LidarFactor* hess) {
        std::vector<OctreeGba*> oct_buf;
        for (OctreeGba* oc : *octs) {
            oc->Recut(hess, &oct_buf);
            oc->CollectLeaves(&oct_buf);
            delete oc;
            for (OctreeGba* oct : oct_buf) {
                delete oct;
            }
            oct_buf.clear();
        }
    };

    std::vector<std::thread> threads;
    for (int i = 1; i < thd_num; ++i) {
        threads.emplace_back(recut_func, &octss[static_cast<size_t>(i)],
                             &vec_voxhess[static_cast<size_t>(i)]);
    }
    recut_func(&octss[0], voxhess);
    for (int i = 1; i < thd_num; ++i) {
        threads[static_cast<size_t>(i - 1)].join();
        voxhess->plvec_voxels.insert(
            voxhess->plvec_voxels.end(),
            vec_voxhess[static_cast<size_t>(i)].plvec_voxels.begin(),
            vec_voxhess[static_cast<size_t>(i)].plvec_voxels.end());
        voxhess->sig_vecs.insert(
            voxhess->sig_vecs.end(),
            vec_voxhess[static_cast<size_t>(i)].sig_vecs.begin(),
            vec_voxhess[static_cast<size_t>(i)].sig_vecs.end());
        voxhess->coeffs.insert(
            voxhess->coeffs.end(),
            vec_voxhess[static_cast<size_t>(i)].coeffs.begin(),
            vec_voxhess[static_cast<size_t>(i)].coeffs.end());
        voxhess->eig_values.insert(
            voxhess->eig_values.end(),
            vec_voxhess[static_cast<size_t>(i)].eig_values.begin(),
            vec_voxhess[static_cast<size_t>(i)].eig_values.end());
        voxhess->eig_vectors.insert(
            voxhess->eig_vectors.end(),
            vec_voxhess[static_cast<size_t>(i)].eig_vectors.begin(),
            vec_voxhess[static_cast<size_t>(i)].eig_vectors.end());
        voxhess->pcr_adds.insert(
            voxhess->pcr_adds.end(),
            vec_voxhess[static_cast<size_t>(i)].pcr_adds.begin(),
            vec_voxhess[static_cast<size_t>(i)].pcr_adds.end());
    }
    feat_map->clear();
}

}  // namespace vxslam
}  // namespace argus

#endif
