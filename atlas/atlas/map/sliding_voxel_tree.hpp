/*
 * Copyright 2026 The Openbot Authors
 * Sliding voxel tree and local map operations.
 */
#ifndef ATLAS_MAP_SLIDING_VOXEL_TREE_HPP_
#define ATLAS_MAP_SLIDING_VOXEL_TREE_HPP_

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <iterator>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "common/math/so3.hpp"
#include "common/pose_state.hpp"
#include "map/fitted_plane.hpp"
#include "map/keyframe.hpp"
#include "map/point_cloud_factor_data.hpp"
#include "map/point_cluster.hpp"
#include "map/point_covariance.hpp"
#include "map/point_observation.hpp"
#include "map/voxel_key_hash.hpp"
#include "sensor/point_types.hpp"

namespace atlas {
namespace map {

using ::atlas::map::FittedPlane;

using ::atlas::map::PointObservation;
using ::atlas::map::PointObservationList;
using ::atlas::map::SharedPointObservationList;

using ::atlas::common::PoseState;

using ::atlas::map::PointCluster;
using ::atlas::map::VoxelKey;

using PointType = sensor::PointType;
using PointCloud = sensor::PointCloudXYZI;

inline Eigen::Matrix3d Exp(const Eigen::Vector3d& a) { return math::Exp(a); }
inline Eigen::Matrix3d Exp(const Eigen::Vector3d& a, double dt) {
  return math::Exp(a, dt);
}
inline Eigen::Vector3d Log(const Eigen::Matrix3d& R) { return math::Log(R); }
inline Eigen::Matrix3d hat(const Eigen::Vector3d& v) {
  return math::SkewSymmetric(v);
}

#ifndef I33
#define I33 Eigen::Matrix3d::Identity()
#endif

struct SlidingVoxelMapOptions {
  double voxel_size = 1.0;
  double minimum_plane_eigenvalue = 0.01;
  int maximum_layer = 2;
  int maximum_points_per_voxel = 100;
  std::vector<double> minimum_point_count_by_layer{5.0, 5.0, 5.0, 5.0};
  std::vector<double> plane_eigenvalue_thresholds;
  std::vector<int> sliding_window_order;

  void AdvanceWindowOrder(int frame_count) {
    if (sliding_window_order.empty()) return;
    const int shift = frame_count % static_cast<int>(sliding_window_order.size());
    std::rotate(sliding_window_order.begin(),
                sliding_window_order.begin() + shift,
                sliding_window_order.end());
  }
};
// The sldingwindow in each voxel nodes
class FrameWindow {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::vector<PointObservationList> points;
  std::vector<PointCluster> pcrs_local;

  FrameWindow(int wdsize) {
    pcrs_local.resize(wdsize);
    points.resize(wdsize);
    for (int i = 0; i < wdsize; i++) points[i].reserve(20);
  }

  void resize(int wdsize) {
    if (points.size() != static_cast<std::size_t>(wdsize)) {
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
class SlidingVoxelTree {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::unique_ptr<FrameWindow> sw;
  PointCluster pcr_add;
  Eigen::Matrix<double, 9, 9> cov_add;

  PointCluster pcr_fix;
  PointObservationList point_fix;

  int layer, octo_state, wdsize;
  std::unique_ptr<SlidingVoxelTree> leaves[8];
  double voxel_center[3];
  double jour = 0;
  float quater_length;

  FittedPlane plane;
  bool isexist = false;
  std::shared_ptr<const SlidingVoxelMapOptions> map_options;

  Eigen::Vector3d eig_value;
  Eigen::Matrix3d eig_vector;

  int last_num = 0, opt_state = -1;
  std::mutex mVox;

  SlidingVoxelTree(int layer_index, int window_size,
                   std::shared_ptr<const SlidingVoxelMapOptions> map_options)
      : layer(layer_index),
        octo_state(0),
        wdsize(window_size),
        map_options(std::move(map_options)) {
    if (this->map_options == nullptr || window_size <= 0 ||
        this->map_options->sliding_window_order.size() <
            static_cast<std::size_t>(wdsize) ||
        !std::isfinite(this->map_options->voxel_size) ||
        this->map_options->voxel_size <= 0.0 ||
        this->map_options->maximum_layer < 0 ||
        this->map_options->plane_eigenvalue_thresholds.size() <=
            static_cast<std::size_t>(this->map_options->maximum_layer) ||
        this->map_options->minimum_point_count_by_layer.size() <=
            static_cast<std::size_t>(this->map_options->maximum_layer)) {
      throw std::invalid_argument("invalid sliding voxel map options");
    }
    for (int mapped_order : this->map_options->sliding_window_order) {
      if (mapped_order < 0 || mapped_order >= wdsize) {
        throw std::invalid_argument("invalid sliding window order");
      }
    }
    cov_add.setZero();

    // ins = 255.0*rand()/(RAND_MAX + 1.0f);
  }

  int WindowIndex(int scan_index) const {
    if (scan_index < 0 || static_cast<std::size_t>(scan_index) >=
                              map_options->sliding_window_order.size()) {
      throw std::out_of_range("sliding window index is out of range");
    }
    return map_options
        ->sliding_window_order[static_cast<std::size_t>(scan_index)];
  }

  inline void push(int ord, const PointObservation& point_observation,
                   const Eigen::Vector3d& pw,
                   std::vector<std::unique_ptr<FrameWindow>>& sws) {
    std::lock_guard<std::mutex> lock(mVox);
    const std::size_t mapped_order = static_cast<std::size_t>(WindowIndex(ord));
    if (sw == nullptr) {
      if (sws.size() != 0) {
        sw = std::move(sws.back());
        sws.pop_back();
        sw->resize(wdsize);
      } else
        sw = std::make_unique<FrameWindow>(wdsize);
    }
    if (!isexist) isexist = true;

    if (layer < map_options->maximum_layer) {
      sw->points[mapped_order].push_back(point_observation);
    }
    sw->pcrs_local[mapped_order].push(point_observation.position);
    pcr_add.push(pw);
    Eigen::Matrix<double, 9, 9> Bi;
    ComputePointTransformCovariance(point_observation.covariance, pw, &Bi);
    cov_add += Bi;
  }

  inline void push_fix(PointObservation& point_observation) {
    if (layer < map_options->maximum_layer)
      point_fix.push_back(point_observation);
    pcr_fix.push(point_observation.position);
    pcr_add.push(point_observation.position);
    Eigen::Matrix<double, 9, 9> Bi;
    ComputePointTransformCovariance(point_observation.covariance,
                                    point_observation.position, &Bi);
    cov_add += Bi;
  }

  inline void push_fix_novar(PointObservation& point_observation) {
    if (layer < map_options->maximum_layer)
      point_fix.push_back(point_observation);
    pcr_fix.push(point_observation.position);
    pcr_add.push(point_observation.position);
  }

  inline bool plane_judge(Eigen::Vector3d& eigenvalues) {
    return (
        eigenvalues[0] < map_options->minimum_plane_eigenvalue &&
        (eigenvalues[0] / eigenvalues[2]) <
            map_options
                ->plane_eigenvalue_thresholds[static_cast<std::size_t>(layer)]);
  }

  void allocate(int ord, const PointObservation& point_observation,
                const Eigen::Vector3d& pw,
                std::vector<std::unique_ptr<FrameWindow>>& sws) {
    if (octo_state == 0) {
      push(ord, point_observation, pw, sws);
    } else {
      int xyz[3] = {0, 0, 0};
      for (int k = 0; k < 3; k++)
        if (pw[k] > voxel_center[k]) xyz[k] = 1;
      int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];

      if (leaves[leafnum] == nullptr) {
        leaves[leafnum] =
            std::make_unique<SlidingVoxelTree>(layer + 1, wdsize, map_options);
        leaves[leafnum]->voxel_center[0] =
            voxel_center[0] + (2 * xyz[0] - 1) * quater_length;
        leaves[leafnum]->voxel_center[1] =
            voxel_center[1] + (2 * xyz[1] - 1) * quater_length;
        leaves[leafnum]->voxel_center[2] =
            voxel_center[2] + (2 * xyz[2] - 1) * quater_length;
        leaves[leafnum]->quater_length = quater_length / 2;
      }

      leaves[leafnum]->allocate(ord, point_observation, pw, sws);
    }
  }

  void allocate_fix(PointObservation& point_observation) {
    if (octo_state == 0) {
      push_fix_novar(point_observation);
    } else if (layer < map_options->maximum_layer) {
      int xyz[3] = {0, 0, 0};
      for (int k = 0; k < 3; k++)
        if (point_observation.position[k] > voxel_center[k]) xyz[k] = 1;
      int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];

      if (leaves[leafnum] == nullptr) {
        leaves[leafnum] =
            std::make_unique<SlidingVoxelTree>(layer + 1, wdsize, map_options);
        leaves[leafnum]->voxel_center[0] =
            voxel_center[0] + (2 * xyz[0] - 1) * quater_length;
        leaves[leafnum]->voxel_center[1] =
            voxel_center[1] + (2 * xyz[1] - 1) * quater_length;
        leaves[leafnum]->voxel_center[2] =
            voxel_center[2] + (2 * xyz[2] - 1) * quater_length;
        leaves[leafnum]->quater_length = quater_length / 2;
      }

      leaves[leafnum]->allocate_fix(point_observation);
    }
  }

  void fix_divide() {
    for (PointObservation& point_observation : point_fix) {
      int xyz[3] = {0, 0, 0};
      for (int k = 0; k < 3; k++)
        if (point_observation.position[k] > voxel_center[k]) xyz[k] = 1;
      int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
      if (leaves[leafnum] == nullptr) {
        leaves[leafnum] =
            std::make_unique<SlidingVoxelTree>(layer + 1, wdsize, map_options);
        leaves[leafnum]->voxel_center[0] =
            voxel_center[0] + (2 * xyz[0] - 1) * quater_length;
        leaves[leafnum]->voxel_center[1] =
            voxel_center[1] + (2 * xyz[1] - 1) * quater_length;
        leaves[leafnum]->voxel_center[2] =
            voxel_center[2] + (2 * xyz[2] - 1) * quater_length;
        leaves[leafnum]->quater_length = quater_length / 2;
      }

      leaves[leafnum]->push_fix(point_observation);
    }
  }

  void subdivide(int si, const PoseState& pose_state,
                 std::vector<std::unique_ptr<FrameWindow>>& sws) {
    for (PointObservation& point_observation : sw->points[WindowIndex(si)]) {
      Eigen::Vector3d pw = pose_state.rotation * point_observation.position +
                           pose_state.translation;
      int xyz[3] = {0, 0, 0};
      for (int k = 0; k < 3; k++)
        if (pw[k] > voxel_center[k]) xyz[k] = 1;
      int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
      if (leaves[leafnum] == nullptr) {
        leaves[leafnum] =
            std::make_unique<SlidingVoxelTree>(layer + 1, wdsize, map_options);
        leaves[leafnum]->voxel_center[0] =
            voxel_center[0] + (2 * xyz[0] - 1) * quater_length;
        leaves[leafnum]->voxel_center[1] =
            voxel_center[1] + (2 * xyz[1] - 1) * quater_length;
        leaves[leafnum]->voxel_center[2] =
            voxel_center[2] + (2 * xyz[2] - 1) * quater_length;
        leaves[leafnum]->quater_length = quater_length / 2;
      }

      leaves[leafnum]->push(si, point_observation, pw, sws);
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
    plane.covariance.block<3, 3>(0, 0) = Jc * u_c.transpose();
    Eigen::Matrix3d Jc_N = nv * Jc.block<3, 3>(0, 6);
    plane.covariance.block<3, 3>(0, 3) = Jc_N;
    plane.covariance.block<3, 3>(3, 0) = Jc_N.transpose();
    plane.covariance.block<3, 3>(3, 3) = nv * nv * cov_add.block<3, 3>(6, 6);
    plane.normal = u[0];
    plane.radius = eig_value[2];
  }

  void Recut(int win_count, const std::vector<PoseState>& pose_window,
             std::vector<std::unique_ptr<FrameWindow>>& sws) {
    if (octo_state == 0) {
      if (layer >= 0) {
        opt_state = -1;
        if (pcr_add.N <=
            map_options->minimum_point_count_by_layer[static_cast<std::size_t>(
                layer)]) {
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
        } else if (layer >= map_options->maximum_layer)
          return;
      }

      if (pcr_fix.N != 0) {
        fix_divide();
        // point_fix.clear();
        PointObservationList().swap(point_fix);
      }

      for (int i = 0; i < win_count; i++)
        subdivide(i, pose_window[static_cast<std::size_t>(i)], sws);

      sw->clear();
      sws.push_back(std::move(sw));
      octo_state = 1;
    }

    for (int i = 0; i < 8; i++)
      if (leaves[i] != nullptr) leaves[i]->Recut(win_count, pose_window, sws);
  }

  void Marginalize(int win_count, int mgsize,
                   const std::vector<PoseState>& pose_window,
                   const PointCloudFactorData& factor_data) {
    if (octo_state == 0 && layer >= 0) {
      if (!isexist || sw == nullptr) return;
      std::lock_guard<std::mutex> lock(mVox);
      std::vector<PointCluster> pcrs_world(wdsize);
      // pcr_add = pcr_fix;
      // for(int i=0; i<win_count; i++)
      // if(sw->pcrs_local[WindowIndex(i)].N != 0)
      // {
      //   pcrs_world[i].transform(sw->pcrs_local[WindowIndex(i)], x_buf[i]);
      //   pcr_add += pcrs_world[i];
      // }

      const bool factor_index_valid =
          opt_state >= 0 &&
          static_cast<std::size_t>(opt_state) <
              factor_data.accumulated_point_clusters.size() &&
          static_cast<std::size_t>(opt_state) <
              factor_data.eigenvalues.size() &&
          static_cast<std::size_t>(opt_state) < factor_data.eigenvectors.size();
      if (factor_index_valid) {
        pcr_add = factor_data.accumulated_point_clusters[opt_state];
        eig_value = factor_data.eigenvalues[opt_state];
        eig_vector = factor_data.eigenvectors[opt_state];
        opt_state = -1;

        for (int i = 0; i < mgsize; i++)
          if (sw->pcrs_local[WindowIndex(i)].N != 0) {
            pcrs_world[i].transform(
                sw->pcrs_local[WindowIndex(i)],
                pose_window[static_cast<std::size_t>(i)].rotation,
                pose_window[static_cast<std::size_t>(i)].translation);
          }
      } else {
        opt_state = -1;
        pcr_add = pcr_fix;
        for (int i = 0; i < win_count; i++)
          if (sw->pcrs_local[WindowIndex(i)].N != 0) {
            pcrs_world[i].transform(
                sw->pcrs_local[WindowIndex(i)],
                pose_window[static_cast<std::size_t>(i)].rotation,
                pose_window[static_cast<std::size_t>(i)].translation);
            pcr_add += pcrs_world[i];
          }

        if (plane.is_plane) {
          Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(pcr_add.cov());
          eig_value = saes.eigenvalues();
          eig_vector = saes.eigenvectors();
        }
      }

      if (pcr_fix.N < map_options->maximum_points_per_voxel && plane.is_plane)
        if (pcr_add.N - last_num >= 5 || last_num <= 10) {
          plane_update();
          last_num = pcr_add.N;
        }

      if (pcr_fix.N < map_options->maximum_points_per_voxel) {
        for (int i = 0; i < mgsize; i++)
          if (pcrs_world[i].N != 0) {
            pcr_fix += pcrs_world[i];
            for (PointObservation point_observation :
                 sw->points[WindowIndex(i)]) {
              point_observation.position =
                  pose_window[static_cast<std::size_t>(i)].rotation *
                      point_observation.position +
                  pose_window[static_cast<std::size_t>(i)].translation;
              point_fix.push_back(point_observation);
            }
          }

      } else {
        for (int i = 0; i < mgsize; i++)
          if (pcrs_world[i].N != 0) pcr_add -= pcrs_world[i];

        if (point_fix.size() != 0) PointObservationList().swap(point_fix);
      }

      for (int i = 0; i < mgsize; i++)
        if (sw->pcrs_local[WindowIndex(i)].N != 0) {
          sw->pcrs_local[WindowIndex(i)].clear();
          sw->points[WindowIndex(i)].clear();
        }

      if (pcr_fix.N >= pcr_add.N)
        isexist = false;
      else
        isexist = true;

    } else {
      isexist = false;
      for (int i = 0; i < 8; i++)
        if (leaves[i] != nullptr) {
          leaves[i]->Marginalize(win_count, mgsize, pose_window, factor_data);
          isexist = isexist || leaves[i]->isexist;
        }
    }
  }

  // Extract the LiDAR factor
  void CollectFactors(PointCloudFactorData& factor_data) {
    if (octo_state == 0) {
      if (layer >= 0 && isexist && plane.is_plane && sw != nullptr) {
        if (eig_value[0] / eig_value[1] > 0.12) return;

        double coe = 1;
        std::vector<PointCluster> pcrs(wdsize);
        for (int i = 0; i < wdsize; i++)
          pcrs[i] = sw->pcrs_local[WindowIndex(i)];
        opt_state = static_cast<int>(factor_data.factor_count());
        factor_data.AddPlaneFactor(pcrs, pcr_fix, coe, eig_value, eig_vector,
                                   pcr_add);
      }

    } else {
      for (int i = 0; i < 8; i++)
        if (leaves[i] != nullptr) leaves[i]->CollectFactors(factor_data);
    }
  }

  int Match(Eigen::Vector3d& wld, FittedPlane*& pla, double& max_prob,
            Eigen::Matrix3d& var_wld, double& sigma_d, SlidingVoxelTree*& oc) {
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
          double sigma_l = J_nq * plane.covariance * J_nq.transpose();
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
      //     leaves[i]->Match(point_observation, pla, max_prob, var_wld);
    }

    return flag;
  }

  // Extract the point cloud map for debug
  void tras_display(int win_count, PointCloud& pl_fixd, PointCloud& pl_wind,
                    const std::vector<PoseState>& pose_window) {
    if (octo_state == 0) {
      PointType ap;
      // ap.intensity = ins;

      if (plane.is_plane) {
        // if(eig_value[0]/eig_value[1] > 0.1)
        //   return;

        // for(PointObservation &point_observation: point_fix)
        // {
        //   Eigen::Vector3d point_observations = point_observation.position;
        //   ap.x = point_observations[0];
        //   ap.y = point_observations[1];
        //   ap.z = point_observations[2];
        //   ap.normal_x = sqrt(eigenvalues[0]);
        //   ap.normal_y = sqrt(eigenvalues[2] / eigenvalues[0]);
        //   ap.normal_z = pcr_add.N;
        //   ap.curvature = pcr_add.N - pcr_fix.N;
        //   pl_fixd.push_back(ap);
        // }

        for (int i = 0; i < win_count; i++)
          for (PointObservation& point_observation :
               sw->points[WindowIndex(i)]) {
            Eigen::Vector3d point_observations =
                pose_window[static_cast<std::size_t>(i)].rotation *
                    point_observation.position +
                pose_window[static_cast<std::size_t>(i)].translation;
            ap.x = point_observations[0];
            ap.y = point_observations[1];
            ap.z = point_observations[2];
            // ap.normal_x = sqrt(eigenvalues[0]);
            // ap.normal_y = sqrt(eigenvalues[2] / eigenvalues[0]);
            // ap.normal_z = pcr_add.N;
            // ap.curvature = pcr_add.N - pcr_fix.N;
            pl_wind.push_back(ap);
          }
      }

    } else {
      for (int i = 0; i < 8; i++)
        if (leaves[i] != nullptr)
          leaves[i]->tras_display(win_count, pl_fixd, pl_wind, pose_window);
    }
  }

  bool inside(Eigen::Vector3d& wld) {
    double hl = quater_length * 2;
    return (wld[0] >= voxel_center[0] - hl && wld[0] <= voxel_center[0] + hl &&
            wld[1] >= voxel_center[1] - hl && wld[1] <= voxel_center[1] + hl &&
            wld[2] >= voxel_center[2] - hl && wld[2] <= voxel_center[2] + hl);
  }

  void clear_slwd(std::vector<std::unique_ptr<FrameWindow>>& sws) {
    if (octo_state != 0) {
      for (int i = 0; i < 8; i++)
        if (leaves[i] != nullptr) {
          leaves[i]->clear_slwd(sws);
        }
    }

    if (sw != nullptr) {
      sw->clear();
      sws.push_back(std::move(sw));
    }
  }
};

using SlidingVoxelMap =
    std::unordered_map<VoxelKey, std::unique_ptr<SlidingVoxelTree>>;
using SlidingVoxelMapView = std::unordered_map<VoxelKey, SlidingVoxelTree*>;

inline void CutVoxel(
    SlidingVoxelMap& feat_map, SharedPointObservationList point_observations,
    int win_count, SlidingVoxelMapView& feat_tem_map, int wdsize,
    AlignedVectorList<3>& pwld, std::vector<std::unique_ptr<FrameWindow>>& sws,
    std::shared_ptr<const SlidingVoxelMapOptions> map_options) {
  if (point_observations == nullptr || map_options == nullptr) {
    return;
  }
  int plsize = point_observations->size();
  for (int i = 0; i < plsize; i++) {
    PointObservation& point_observation = (*point_observations)[i];
    Eigen::Vector3d& pw = pwld[i];
    float loc[3];
    for (int j = 0; j < 3; j++) {
      loc[j] = pw[j] / map_options->voxel_size;
      if (loc[j] < 0) loc[j] -= 1;
    }

    VoxelKey position(loc[0], loc[1], loc[2]);
    auto iter = feat_map.find(position);
    if (iter != feat_map.end()) {
      iter->second->allocate(win_count, point_observation, pw, sws);
      iter->second->isexist = true;
      if (feat_tem_map.find(position) == feat_tem_map.end())
        feat_tem_map[position] = iter->second.get();
    } else {
      auto ot = std::make_unique<SlidingVoxelTree>(0, wdsize, map_options);
      ot->allocate(win_count, point_observation, pw, sws);
      ot->voxel_center[0] = (0.5 + position.x) * map_options->voxel_size;
      ot->voxel_center[1] = (0.5 + position.y) * map_options->voxel_size;
      ot->voxel_center[2] = (0.5 + position.z) * map_options->voxel_size;
      ot->quater_length = map_options->voxel_size / 4.0;
      feat_tem_map[position] = ot.get();
      feat_map[position] = std::move(ot);
    }
  }
}

inline void CutVoxel(
    SlidingVoxelMap& feat_map, PointObservationList& point_observations,
    int wdsize, double jour,
    std::shared_ptr<const SlidingVoxelMapOptions> map_options) {
  if (map_options == nullptr) {
    return;
  }
  for (PointObservation& point_observation : point_observations) {
    float loc[3];
    for (int j = 0; j < 3; j++) {
      loc[j] = point_observation.position[j] / map_options->voxel_size;
      if (loc[j] < 0) loc[j] -= 1;
    }

    VoxelKey position(loc[0], loc[1], loc[2]);
    auto iter = feat_map.find(position);
    if (iter != feat_map.end()) {
      iter->second->allocate_fix(point_observation);
    } else {
      auto ot = std::make_unique<SlidingVoxelTree>(0, wdsize, map_options);
      ot->push_fix_novar(point_observation);
      ot->voxel_center[0] = (0.5 + position.x) * map_options->voxel_size;
      ot->voxel_center[1] = (0.5 + position.y) * map_options->voxel_size;
      ot->voxel_center[2] = (0.5 + position.z) * map_options->voxel_size;
      ot->quater_length = map_options->voxel_size / 4.0;
      ot->jour = jour;
      feat_map[position] = std::move(ot);
    }
  }
}

// Match the point with the plane in the voxel map
inline int Match(SlidingVoxelMap& feat_map, Eigen::Vector3d& wld,
                 FittedPlane*& pla, Eigen::Matrix3d& var_wld, double& sigma_d,
                 SlidingVoxelTree*& oc,
                 std::shared_ptr<const SlidingVoxelMapOptions> map_options) {
  if (map_options == nullptr) {
    return 0;
  }
  int flag = 0;

  float loc[3];
  for (int j = 0; j < 3; j++) {
    loc[j] = wld[j] / map_options->voxel_size;
    if (loc[j] < 0) loc[j] -= 1;
  }
  VoxelKey position(loc[0], loc[1], loc[2]);
  auto iter = feat_map.find(position);
  if (iter != feat_map.end()) {
    double max_prob = 0;
    flag = iter->second->Match(wld, pla, max_prob, var_wld, sigma_d, oc);
    // iter->second->match_end(point_observation, pla, max_prob);
    if (flag && pla == nullptr) {
      printf("pla null max_prob: %lf %" PRId64 " %" PRId64 " %" PRId64 "\n",
             max_prob, iter->first.x, iter->first.y, iter->first.z);
    }
  }

  return flag;
}

}  // namespace map
}  // namespace atlas

#endif  // ATLAS_MAP_SLIDING_VOXEL_TREE_HPP_
