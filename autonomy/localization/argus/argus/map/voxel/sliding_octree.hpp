/*
 * Copyright 2026 The Openbot Authors
 * Sliding voxel octree + local map ops (Voxel-SLAM SlidingOctree port).
 */
#ifndef ARGUS_VXSLAM_SLIDING_OCTREE_HPP_
#define ARGUS_VXSLAM_SLIDING_OCTREE_HPP_

#include <Eigen/Eigenvalues>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "common/math/so3.hpp"
#include "frontend/lidar/imu/imu_preintegration.hpp"
#include "frontend/lidar/imu/imu_state.hpp"
#include "frontend/lidar/local_mapping/lidar_factor.hpp"
#include "frontend/lidar/odometry/point_var.hpp"
#include "map/voxel/keyframe.hpp"
#include "map/voxel/point_cluster.hpp"
#include "map/voxel/voxel_key_hash.hpp"
#include "sensor/point_types.hpp"

namespace argus {
namespace vxslam {

using map::voxel::VoxelKey;

using PointType = sensor::PointType;
using PointCloud = sensor::PointCloudXYZI;

#ifndef ARGUS_VXSLAM_MATH_ALIASES_
#define ARGUS_VXSLAM_MATH_ALIASES_
inline Eigen::Matrix3d Exp(const Eigen::Vector3d& a) { return math::Exp(a); }
inline Eigen::Matrix3d Exp(const Eigen::Vector3d& a, double dt) {
  return math::Exp(a, dt);
}
inline Eigen::Vector3d Log(const Eigen::Matrix3d& R) { return math::Log(R); }
inline Eigen::Matrix3d hat(const Eigen::Vector3d& v) {
  return math::SkewSymmetric(v);
}
#endif
#ifndef I33
#define I33 Eigen::Matrix3d::Identity()
#endif
#ifndef ARGUS_VXSLAM_DIM_DEFINED_
#define ARGUS_VXSLAM_DIM_DEFINED_
constexpr int DIM = kStateDim;
#endif

inline Eigen::Vector4d min_point;
inline double min_eigen_value = 0.01;
inline int max_layer = 2;
inline int max_points = 100;
inline double voxel_size = 1.0;
inline int min_ba_point = 20;
inline std::vector<double> plane_eigenvalue_thresholds;
// The sldingwindow in each voxel nodes
class FrameWindow {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::vector<PointVarList> points;
  std::vector<PointCluster> pcrs_local;

  FrameWindow(int wdsize) {
    pcrs_local.resize(wdsize);
    points.resize(wdsize);
    for (int i = 0; i < wdsize; i++) points[i].reserve(20);
  }

  void resize(int wdsize) {
    if (points.size() != wdsize) {
      points.resize(wdsize);
      pcrs_local.resize(wdsize);
    }
  }

  void clear() {
    int wdsize = points.size();
    for (int i = 0; i < wdsize; i++) {
      points[i].clear();
      pcrs_local[i].clear();
    }
  }
};

// The octotree map for odometry and local mapping
// You can re-write it in your own project
inline int* mp = nullptr;
class SlidingOctree {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FrameWindow* sw = nullptr;
  PointCluster pcr_add;
  Eigen::Matrix<double, 9, 9> cov_add;

  PointCluster pcr_fix;
  PointVarList point_fix;

  int layer, octo_state, wdsize;
  SlidingOctree* leaves[8];
  double voxel_center[3];
  double jour = 0;
  float quater_length;

  Plane plane;
  bool isexist = false;

  Eigen::Vector3d eig_value;
  Eigen::Matrix3d eig_vector;

  int last_num = 0, opt_state = -1;
  std::mutex mVox;

  SlidingOctree(int _l, int _w) : layer(_l), wdsize(_w), octo_state(0) {
    for (int i = 0; i < 8; i++) leaves[i] = nullptr;
    cov_add.setZero();

    // ins = 255.0*rand()/(RAND_MAX + 1.0f);
  }

  inline void push(int ord, const PointVar& pv, const Eigen::Vector3d& pw,
                   std::vector<FrameWindow*>& sws) {
    mVox.lock();
    if (sw == nullptr) {
      if (sws.size() != 0) {
        sw = sws.back();
        sws.pop_back();
        sw->resize(wdsize);
      } else
        sw = new FrameWindow(wdsize);
    }
    if (!isexist) isexist = true;

    int mord = mp[ord];
    if (layer < max_layer) sw->points[mord].push_back(pv);
    sw->pcrs_local[mord].push(pv.pnt);
    pcr_add.push(pw);
    Eigen::Matrix<double, 9, 9> Bi;
    Bf_var(pv, Bi, pw);
    cov_add += Bi;
    mVox.unlock();
  }

  inline void push_fix(PointVar& pv) {
    if (layer < max_layer) point_fix.push_back(pv);
    pcr_fix.push(pv.pnt);
    pcr_add.push(pv.pnt);
    Eigen::Matrix<double, 9, 9> Bi;
    Bf_var(pv, Bi, pv.pnt);
    cov_add += Bi;
  }

  inline void push_fix_novar(PointVar& pv) {
    if (layer < max_layer) point_fix.push_back(pv);
    pcr_fix.push(pv.pnt);
    pcr_add.push(pv.pnt);
  }

  inline bool plane_judge(Eigen::Vector3d& eig_values) {
    // return (eig_values[0] < min_eigen_value);
    return (eig_values[0] < min_eigen_value &&
            (eig_values[0] / eig_values[2]) <
                plane_eigenvalue_thresholds[layer]);
  }

  void allocate(int ord, const PointVar& pv, const Eigen::Vector3d& pw,
                std::vector<FrameWindow*>& sws) {
    if (octo_state == 0) {
      push(ord, pv, pw, sws);
    } else {
      int xyz[3] = {0, 0, 0};
      for (int k = 0; k < 3; k++)
        if (pw[k] > voxel_center[k]) xyz[k] = 1;
      int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];

      if (leaves[leafnum] == nullptr) {
        leaves[leafnum] = new SlidingOctree(layer + 1, wdsize);
        leaves[leafnum]->voxel_center[0] =
            voxel_center[0] + (2 * xyz[0] - 1) * quater_length;
        leaves[leafnum]->voxel_center[1] =
            voxel_center[1] + (2 * xyz[1] - 1) * quater_length;
        leaves[leafnum]->voxel_center[2] =
            voxel_center[2] + (2 * xyz[2] - 1) * quater_length;
        leaves[leafnum]->quater_length = quater_length / 2;
      }

      leaves[leafnum]->allocate(ord, pv, pw, sws);
    }
  }

  void allocate_fix(PointVar& pv) {
    if (octo_state == 0) {
      push_fix_novar(pv);
    } else if (layer < max_layer) {
      int xyz[3] = {0, 0, 0};
      for (int k = 0; k < 3; k++)
        if (pv.pnt[k] > voxel_center[k]) xyz[k] = 1;
      int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];

      if (leaves[leafnum] == nullptr) {
        leaves[leafnum] = new SlidingOctree(layer + 1, wdsize);
        leaves[leafnum]->voxel_center[0] =
            voxel_center[0] + (2 * xyz[0] - 1) * quater_length;
        leaves[leafnum]->voxel_center[1] =
            voxel_center[1] + (2 * xyz[1] - 1) * quater_length;
        leaves[leafnum]->voxel_center[2] =
            voxel_center[2] + (2 * xyz[2] - 1) * quater_length;
        leaves[leafnum]->quater_length = quater_length / 2;
      }

      leaves[leafnum]->allocate_fix(pv);
    }
  }

  void fix_divide(std::vector<FrameWindow*>& sws) {
    for (PointVar& pv : point_fix) {
      int xyz[3] = {0, 0, 0};
      for (int k = 0; k < 3; k++)
        if (pv.pnt[k] > voxel_center[k]) xyz[k] = 1;
      int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
      if (leaves[leafnum] == nullptr) {
        leaves[leafnum] = new SlidingOctree(layer + 1, wdsize);
        leaves[leafnum]->voxel_center[0] =
            voxel_center[0] + (2 * xyz[0] - 1) * quater_length;
        leaves[leafnum]->voxel_center[1] =
            voxel_center[1] + (2 * xyz[1] - 1) * quater_length;
        leaves[leafnum]->voxel_center[2] =
            voxel_center[2] + (2 * xyz[2] - 1) * quater_length;
        leaves[leafnum]->quater_length = quater_length / 2;
      }

      leaves[leafnum]->push_fix(pv);
    }
  }

  void subdivide(int si, ImuState& xx, std::vector<FrameWindow*>& sws) {
    for (PointVar& pv : sw->points[mp[si]]) {
      Eigen::Vector3d pw = xx.R * pv.pnt + xx.p;
      int xyz[3] = {0, 0, 0};
      for (int k = 0; k < 3; k++)
        if (pw[k] > voxel_center[k]) xyz[k] = 1;
      int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
      if (leaves[leafnum] == nullptr) {
        leaves[leafnum] = new SlidingOctree(layer + 1, wdsize);
        leaves[leafnum]->voxel_center[0] =
            voxel_center[0] + (2 * xyz[0] - 1) * quater_length;
        leaves[leafnum]->voxel_center[1] =
            voxel_center[1] + (2 * xyz[1] - 1) * quater_length;
        leaves[leafnum]->voxel_center[2] =
            voxel_center[2] + (2 * xyz[2] - 1) * quater_length;
        leaves[leafnum]->quater_length = quater_length / 2;
      }

      leaves[leafnum]->push(si, pv, pw, sws);
    }
  }

  void plane_update() {
    plane.center = pcr_add.v / pcr_add.N;
    int l = 0;
    Eigen::Vector3d u[3] = {eig_vector.col(0), eig_vector.col(1),
                            eig_vector.col(2)};
    double nv = 1.0 / pcr_add.N;

    Eigen::Matrix<double, 3, 9> u_c;
    u_c.setZero();
    for (int k = 0; k < 3; k++)
      if (k != l) {
        Eigen::Matrix3d ukl = u[k] * u[l].transpose();
        Eigen::Matrix<double, 1, 9> fkl;
        fkl.head(6) << ukl(0, 0), ukl(1, 0) + ukl(0, 1), ukl(2, 0) + ukl(0, 2),
            ukl(1, 1), ukl(1, 2) + ukl(2, 1), ukl(2, 2);
        fkl.tail(3) =
            -(u[k].dot(plane.center) * u[l] + u[l].dot(plane.center) * u[k]);

        u_c += nv / (eig_value[l] - eig_value[k]) * u[k] * fkl;
      }

    Eigen::Matrix<double, 3, 9> Jc = u_c * cov_add;
    plane.plane_var.block<3, 3>(0, 0) = Jc * u_c.transpose();
    Eigen::Matrix3d Jc_N = nv * Jc.block<3, 3>(0, 6);
    plane.plane_var.block<3, 3>(0, 3) = Jc_N;
    plane.plane_var.block<3, 3>(3, 0) = Jc_N.transpose();
    plane.plane_var.block<3, 3>(3, 3) = nv * nv * cov_add.block<3, 3>(6, 6);
    plane.normal = u[0];
    plane.radius = eig_value[2];
  }

  void Recut(int win_count, std::vector<ImuState>& x_buf,
             std::vector<FrameWindow*>& sws) {
    if (octo_state == 0) {
      if (layer >= 0) {
        opt_state = -1;
        if (pcr_add.N <= min_point[layer]) {
          plane.is_plane = false;
          return;
        }
        if (!isexist || sw == nullptr) return;

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(pcr_add.cov());
        eig_value = saes.eigenvalues();
        eig_vector = saes.eigenvectors();
        plane.is_plane = plane_judge(eig_value);

        if (plane.is_plane) {
          return;
        } else if (layer >= max_layer)
          return;
      }

      if (pcr_fix.N != 0) {
        fix_divide(sws);
        // point_fix.clear();
        PointVarList().swap(point_fix);
      }

      for (int i = 0; i < win_count; i++) subdivide(i, x_buf[i], sws);

      sw->clear();
      sws.push_back(sw);
      sw = nullptr;
      octo_state = 1;
    }

    for (int i = 0; i < 8; i++)
      if (leaves[i] != nullptr) leaves[i]->Recut(win_count, x_buf, sws);
  }

  void Marginalize(int win_count, int mgsize, std::vector<ImuState>& x_buf,
                   const LidarFactor& vox_opt) {
    if (octo_state == 0 && layer >= 0) {
      if (!isexist || sw == nullptr) return;
      mVox.lock();
      std::vector<PointCluster> pcrs_world(wdsize);
      // pcr_add = pcr_fix;
      // for(int i=0; i<win_count; i++)
      // if(sw->pcrs_local[mp[i]].N != 0)
      // {
      //   pcrs_world[i].transform(sw->pcrs_local[mp[i]], x_buf[i]);
      //   pcr_add += pcrs_world[i];
      // }

      if (opt_state >= int(vox_opt.pcr_adds.size())) {
        printf("Error: opt_state: %d %zu\n", opt_state,
               vox_opt.pcr_adds.size());
        return;
      }

      if (opt_state >= 0) {
        pcr_add = vox_opt.pcr_adds[opt_state];
        eig_value = vox_opt.eig_values[opt_state];
        eig_vector = vox_opt.eig_vectors[opt_state];
        opt_state = -1;

        for (int i = 0; i < mgsize; i++)
          if (sw->pcrs_local[mp[i]].N != 0) {
            pcrs_world[i].transform(sw->pcrs_local[mp[i]], x_buf[i]);
          }
      } else {
        pcr_add = pcr_fix;
        for (int i = 0; i < win_count; i++)
          if (sw->pcrs_local[mp[i]].N != 0) {
            pcrs_world[i].transform(sw->pcrs_local[mp[i]], x_buf[i]);
            pcr_add += pcrs_world[i];
          }

        if (plane.is_plane) {
          Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(pcr_add.cov());
          eig_value = saes.eigenvalues();
          eig_vector = saes.eigenvectors();
        }
      }

      if (pcr_fix.N < max_points && plane.is_plane)
        if (pcr_add.N - last_num >= 5 || last_num <= 10) {
          plane_update();
          last_num = pcr_add.N;
        }

      if (pcr_fix.N < max_points) {
        for (int i = 0; i < mgsize; i++)
          if (pcrs_world[i].N != 0) {
            pcr_fix += pcrs_world[i];
            for (PointVar pv : sw->points[mp[i]]) {
              pv.pnt = x_buf[i].R * pv.pnt + x_buf[i].p;
              point_fix.push_back(pv);
            }
          }

      } else {
        for (int i = 0; i < mgsize; i++)
          if (pcrs_world[i].N != 0) pcr_add -= pcrs_world[i];

        if (point_fix.size() != 0) PointVarList().swap(point_fix);
      }

      for (int i = 0; i < mgsize; i++)
        if (sw->pcrs_local[mp[i]].N != 0) {
          sw->pcrs_local[mp[i]].clear();
          sw->points[mp[i]].clear();
        }

      if (pcr_fix.N >= pcr_add.N)
        isexist = false;
      else
        isexist = true;

      mVox.unlock();
    } else {
      isexist = false;
      for (int i = 0; i < 8; i++)
        if (leaves[i] != nullptr) {
          leaves[i]->Marginalize(win_count, mgsize, x_buf, vox_opt);
          isexist = isexist || leaves[i]->isexist;
        }
    }
  }

  // Extract the LiDAR factor
  void CollectFactors(LidarFactor& vox_opt) {
    if (octo_state == 0) {
      if (layer >= 0 && isexist && plane.is_plane && sw != nullptr) {
        if (eig_value[0] / eig_value[1] > 0.12) return;

        double coe = 1;
        std::vector<PointCluster> pcrs(wdsize);
        for (int i = 0; i < wdsize; i++) pcrs[i] = sw->pcrs_local[mp[i]];
        opt_state = vox_opt.plvec_voxels.size();
        vox_opt.push_voxel(pcrs, pcr_fix, coe, eig_value, eig_vector, pcr_add);
      }

    } else {
      for (int i = 0; i < 8; i++)
        if (leaves[i] != nullptr) leaves[i]->CollectFactors(vox_opt);
    }
  }

  int Match(Eigen::Vector3d& wld, Plane*& pla, double& max_prob,
            Eigen::Matrix3d& var_wld, double& sigma_d, SlidingOctree*& oc) {
    int flag = 0;
    if (octo_state == 0) {
      if (plane.is_plane) {
        float dis_to_plane = fabs(plane.normal.dot(wld - plane.center));
        float dis_to_center = (plane.center - wld).squaredNorm();
        float range_dis = (dis_to_center - dis_to_plane * dis_to_plane);
        if (range_dis <= 3 * 3 * plane.radius) {
          Eigen::Matrix<double, 1, 6> J_nq;
          J_nq.block<1, 3>(0, 0) = wld - plane.center;
          J_nq.block<1, 3>(0, 3) = -plane.normal;
          double sigma_l = J_nq * plane.plane_var * J_nq.transpose();
          sigma_l += plane.normal.transpose() * var_wld * plane.normal;
          if (dis_to_plane < 3 * sqrt(sigma_l)) {
            // float prob = 1 / (sqrt(sigma_l)) * exp(-0.5 * dis_to_plane *
            // dis_to_plane / sigma_l); if(prob > max_prob)
            {
              oc = this;
              sigma_d = sigma_l;
              // max_prob = prob;
              pla = &plane;
            }

            flag = 1;
          }
        }
      }
    } else {
      int xyz[3] = {0, 0, 0};
      for (int k = 0; k < 3; k++)
        if (wld[k] > voxel_center[k]) xyz[k] = 1;
      int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];

      // for(int i=0; i<8; i++)
      // if(leaves[i] != nullptr)
      // {
      //   int flg = leaves[i]->Match(wld, pla, max_prob, var_wld);
      //   if(i == leafnum)
      //     flag = flg;
      // }

      if (leaves[leafnum] != nullptr)
        flag = leaves[leafnum]->Match(wld, pla, max_prob, var_wld, sigma_d, oc);

      // for(int i=0; i<8; i++)
      //   if(leaves[i] != nullptr)
      //     leaves[i]->Match(pv, pla, max_prob, var_wld);
    }

    return flag;
  }

  void tras_ptr(std::vector<SlidingOctree*>& octos_release) {
    if (octo_state == 1) {
      for (int i = 0; i < 8; i++)
        if (leaves[i] != nullptr) {
          octos_release.push_back(leaves[i]);
          leaves[i]->tras_ptr(octos_release);
        }
    }
  }

  // ~SlidingOctree()
  // {
  //   for(int i=0; i<8; i++)
  //   if(leaves[i] != nullptr)
  //   {
  //     delete leaves[i];
  //     leaves[i] = nullptr;
  //   }
  // }

  // Extract the point cloud map for debug
  void tras_display(int win_count, PointCloud& pl_fixd, PointCloud& pl_wind,
                    std::vector<ImuState>& x_buf) {
    if (octo_state == 0) {
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(pcr_add.cov());
      Eigen::Matrix3d eig_vectors = saes.eigenvectors();
      Eigen::Vector3d eig_values = saes.eigenvalues();

      PointType ap;
      // ap.intensity = ins;

      if (plane.is_plane) {
        // if(pcr_add.N-pcr_fix.N < min_ba_point) return;
        // if(eig_value[0]/eig_value[1] > 0.1)
        //   return;

        // for(PointVar &pv: point_fix)
        // {
        //   Eigen::Vector3d pvec = pv.pnt;
        //   ap.x = pvec[0];
        //   ap.y = pvec[1];
        //   ap.z = pvec[2];
        //   ap.normal_x = sqrt(eig_values[0]);
        //   ap.normal_y = sqrt(eig_values[2] / eig_values[0]);
        //   ap.normal_z = pcr_add.N;
        //   ap.curvature = pcr_add.N - pcr_fix.N;
        //   pl_fixd.push_back(ap);
        // }

        for (int i = 0; i < win_count; i++)
          for (PointVar& pv : sw->points[mp[i]]) {
            Eigen::Vector3d pvec = x_buf[i].R * pv.pnt + x_buf[i].p;
            ap.x = pvec[0];
            ap.y = pvec[1];
            ap.z = pvec[2];
            // ap.normal_x = sqrt(eig_values[0]);
            // ap.normal_y = sqrt(eig_values[2] / eig_values[0]);
            // ap.normal_z = pcr_add.N;
            // ap.curvature = pcr_add.N - pcr_fix.N;
            pl_wind.push_back(ap);
          }
      }

    } else {
      for (int i = 0; i < 8; i++)
        if (leaves[i] != nullptr)
          leaves[i]->tras_display(win_count, pl_fixd, pl_wind, x_buf);
    }
  }

  bool inside(Eigen::Vector3d& wld) {
    double hl = quater_length * 2;
    return (wld[0] >= voxel_center[0] - hl && wld[0] <= voxel_center[0] + hl &&
            wld[1] >= voxel_center[1] - hl && wld[1] <= voxel_center[1] + hl &&
            wld[2] >= voxel_center[2] - hl && wld[2] <= voxel_center[2] + hl);
  }

  void clear_slwd(std::vector<FrameWindow*>& sws) {
    if (octo_state != 0) {
      for (int i = 0; i < 8; i++)
        if (leaves[i] != nullptr) {
          leaves[i]->clear_slwd(sws);
        }
    }

    if (sw != nullptr) {
      sw->clear();
      sws.push_back(sw);
      sw = nullptr;
    }
  }
};

inline void CutVoxel(std::unordered_map<VoxelKey, SlidingOctree*>& feat_map,
                     PointVarListPtr pvec, int win_count,
                     std::unordered_map<VoxelKey, SlidingOctree*>& feat_tem_map,
                     int wdsize, VectorList<3>& pwld,
                     std::vector<FrameWindow*>& sws) {
  int plsize = pvec->size();
  for (int i = 0; i < plsize; i++) {
    PointVar& pv = (*pvec)[i];
    Eigen::Vector3d& pw = pwld[i];
    float loc[3];
    for (int j = 0; j < 3; j++) {
      loc[j] = pw[j] / voxel_size;
      if (loc[j] < 0) loc[j] -= 1;
    }

    VoxelKey position(loc[0], loc[1], loc[2]);
    auto iter = feat_map.find(position);
    if (iter != feat_map.end()) {
      iter->second->allocate(win_count, pv, pw, sws);
      iter->second->isexist = true;
      if (feat_tem_map.find(position) == feat_map.end())
        feat_tem_map[position] = iter->second;
    } else {
      SlidingOctree* ot = new SlidingOctree(0, wdsize);
      ot->allocate(win_count, pv, pw, sws);
      ot->voxel_center[0] = (0.5 + position.x) * voxel_size;
      ot->voxel_center[1] = (0.5 + position.y) * voxel_size;
      ot->voxel_center[2] = (0.5 + position.z) * voxel_size;
      ot->quater_length = voxel_size / 4.0;
      feat_map[position] = ot;
      feat_tem_map[position] = ot;
    }
  }
}

// Cut the current scan into corresponding voxel in multi std::thread
inline void CutVoxel_multi(
    std::unordered_map<VoxelKey, SlidingOctree*>& feat_map,
    PointVarListPtr pvec, int win_count,
    std::unordered_map<VoxelKey, SlidingOctree*>& feat_tem_map, int wdsize,
    VectorList<3>& pwld, std::vector<std::vector<FrameWindow*>>& sws) {
  std::unordered_map<SlidingOctree*, std::vector<int>> map_pvec;
  int plsize = pvec->size();
  for (int i = 0; i < plsize; i++) {
    PointVar& pv = (*pvec)[i];
    Eigen::Vector3d& pw = pwld[i];
    float loc[3];
    for (int j = 0; j < 3; j++) {
      // loc[j] = pv.world[j] / voxel_size;
      loc[j] = pw[j] / voxel_size;
      if (loc[j] < 0) loc[j] -= 1;
    }

    VoxelKey position(loc[0], loc[1], loc[2]);
    auto iter = feat_map.find(position);
    SlidingOctree* ot = nullptr;
    if (iter != feat_map.end()) {
      iter->second->isexist = true;
      if (feat_tem_map.find(position) == feat_map.end())
        feat_tem_map[position] = iter->second;
      ot = iter->second;
    } else {
      ot = new SlidingOctree(0, wdsize);
      ot->voxel_center[0] = (0.5 + position.x) * voxel_size;
      ot->voxel_center[1] = (0.5 + position.y) * voxel_size;
      ot->voxel_center[2] = (0.5 + position.z) * voxel_size;
      ot->quater_length = voxel_size / 4.0;
      feat_map[position] = ot;
      feat_tem_map[position] = ot;
    }

    map_pvec[ot].push_back(i);
  }

  // for(auto iter=map_pvec.begin(); iter!=map_pvec.end(); iter++)
  // {
  //   for(int i: iter->second)
  //   {
  //     iter->first->allocate(win_count, (*pvec)[i], pwld[i], sws);
  //   }
  // }

  std::vector<std::pair<SlidingOctree* const, std::vector<int>>*> octs;
  octs.reserve(map_pvec.size());
  for (auto iter = map_pvec.begin(); iter != map_pvec.end(); iter++)
    octs.push_back(&(*iter));

  int thd_num = sws.size();
  int g_size = octs.size();
  if (g_size < thd_num) return;
  std::vector<std::thread*> mthreads(thd_num);
  double part = 1.0 * g_size / thd_num;

  int swsize = sws[0].size() / thd_num;
  for (int i = 1; i < thd_num; i++) {
    sws[i].insert(sws[i].end(), sws[0].end() - swsize, sws[0].end());
    sws[0].erase(sws[0].end() - swsize, sws[0].end());
  }

  for (int i = 1; i < thd_num; i++) {
    mthreads[i] = new std::thread(
        [&](int head, int tail, std::vector<FrameWindow*>& sw) {
          for (int j = head; j < tail; j++) {
            for (int k : octs[j]->second)
              octs[j]->first->allocate(win_count, (*pvec)[k], pwld[k], sw);
          }
        },
        part * i, part * (i + 1), std::ref(sws[i]));
  }

  for (int i = 0; i < thd_num; i++) {
    if (i == 0) {
      for (int j = 0; j < int(part); j++)
        for (int k : octs[j]->second)
          octs[j]->first->allocate(win_count, (*pvec)[k], pwld[k], sws[0]);
    } else {
      mthreads[i]->join();
      delete mthreads[i];
    }
  }
}

inline void CutVoxel(std::unordered_map<VoxelKey, SlidingOctree*>& feat_map,
                     PointVarList& pvec, int wdsize, double jour) {
  for (PointVar& pv : pvec) {
    float loc[3];
    for (int j = 0; j < 3; j++) {
      loc[j] = pv.pnt[j] / voxel_size;
      if (loc[j] < 0) loc[j] -= 1;
    }

    VoxelKey position(loc[0], loc[1], loc[2]);
    auto iter = feat_map.find(position);
    if (iter != feat_map.end()) {
      iter->second->allocate_fix(pv);
    } else {
      SlidingOctree* ot = new SlidingOctree(0, wdsize);
      ot->push_fix_novar(pv);
      ot->voxel_center[0] = (0.5 + position.x) * voxel_size;
      ot->voxel_center[1] = (0.5 + position.y) * voxel_size;
      ot->voxel_center[2] = (0.5 + position.z) * voxel_size;
      ot->quater_length = voxel_size / 4.0;
      ot->jour = jour;
      feat_map[position] = ot;
    }
  }
}

// Match the point with the plane in the voxel map
inline int Match(std::unordered_map<VoxelKey, SlidingOctree*>& feat_map,
                 Eigen::Vector3d& wld, Plane*& pla, Eigen::Matrix3d& var_wld,
                 double& sigma_d, SlidingOctree*& oc) {
  int flag = 0;

  float loc[3];
  for (int j = 0; j < 3; j++) {
    loc[j] = wld[j] / voxel_size;
    if (loc[j] < 0) loc[j] -= 1;
  }
  VoxelKey position(loc[0], loc[1], loc[2]);
  auto iter = feat_map.find(position);
  if (iter != feat_map.end()) {
    double max_prob = 0;
    flag = iter->second->Match(wld, pla, max_prob, var_wld, sigma_d, oc);
    // iter->second->match_end(pv, pla, max_prob);
    if (flag && pla == nullptr) {
      printf("pla null max_prob: %lf %" PRId64 " %" PRId64 " %" PRId64 "\n",
             max_prob, iter->first.x, iter->first.y, iter->first.z);
    }
  }

  return flag;
}

}  // namespace vxslam
}  // namespace argus

#endif  // ARGUS_VXSLAM_SLIDING_OCTREE_HPP_
