/*
 * Copyright 2026 The Openbot Authors
 */
#include "frontend/lidar/initialization/initialization.hpp"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>

#include "common/math/so3.hpp"
#include "frontend/lidar/local_mapping/li_ba_optimizer.hpp"
#include "frontend/lidar/odometry/point_cov.hpp"
#include "glog/logging.h"

namespace atlas {
namespace vxslam {
namespace {

struct ScopedFrontendVoxelParameters {
  double min_eigen_value = vxslam::min_eigen_value;
  std::vector<double> plane_thresholds = vxslam::plane_eigenvalue_thresholds;

  ~ScopedFrontendVoxelParameters() {
    vxslam::min_eigen_value = min_eigen_value;
    vxslam::plane_eigenvalue_thresholds.swap(plane_thresholds);
  }
};

}  // namespace

bool AlignGravity(const std::deque<sensor::ImuData>& imus, ImuState* x,
                  double scale_gravity) {
  if (x == nullptr || imus.size() < 10) {
    return false;
  }
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (const auto& imu : imus) {
    mean += imu.linear_acceleration;
  }
  mean /= static_cast<double>(imus.size());
  x->SetZero();
  x->g = -mean * scale_gravity;
  return true;
}

bool MotionInitReady(int buffered_scans, int min_scans) {
  return buffered_scans >= min_scans;
}

void AlignGravityStates(std::vector<ImuState>* xs) {
  if (xs == nullptr || xs->empty()) {
    return;
  }
  Eigen::Vector3d g0 = (*xs)[0].g;
  Eigen::Vector3d n0 = g0 / g0.norm();
  Eigen::Vector3d n1(0, 0, 1);
  if (n0[2] < 0) {
    n1[2] = -1;
  }
  Eigen::Vector3d rotvec = n0.cross(n1);
  const double rnorm = rotvec.norm();
  if (rnorm < 1e-12) {
    return;
  }
  rotvec /= rnorm;
  const Eigen::AngleAxisd angaxis(std::asin(std::min(1.0, rnorm)), rotvec);
  const Eigen::Matrix3d rot = angaxis.matrix();
  g0 = rot * g0;
  const Eigen::Vector3d p0 = (*xs)[0].p;
  for (ImuState& x : *xs) {
    x.p = rot * (x.p - p0) + p0;
    x.R = rot * x.R;
    x.v = rot * x.v;
    x.g = g0;
  }
}

void MotionBlur(const sensor::PointCloudXYZI& pl, PointVarList* pvec,
                ImuState xc, const ImuState& xl,
                const std::deque<sensor::ImuData>& imus, double pcl_beg_time,
                const ImuState& extrin, bool point_notime) {
  if (pvec == nullptr) {
    return;
  }
  pvec->clear();
  xc.bg = xl.bg;
  xc.ba = xl.ba;
  Eigen::Vector3d acc_imu;
  Eigen::Vector3d angvel_avr;
  Eigen::Vector3d acc_avr;
  Eigen::Vector3d vel_imu(xc.v);
  Eigen::Vector3d pos_imu(xc.p);
  Eigen::Matrix3d R_imu(xc.R);
  std::vector<ImuState> imu_poses;

  if (imus.size() < 2) {
    PointVar pv;
    pv.var.setIdentity();
    for (const sensor::PointType& ap : pl.points) {
      pv.pnt << ap.x, ap.y, ap.z;
      pv.pnt = extrin.R * pv.pnt + extrin.p;
      pvec->push_back(pv);
    }
    return;
  }

  for (auto it = imus.end() - 1; it != imus.begin(); --it) {
    const sensor::ImuData& head = *(it - 1);
    const sensor::ImuData& tail = *it;
    angvel_avr = 0.5 * (head.angular_velocity + tail.angular_velocity);
    acc_avr = 0.5 * (head.linear_acceleration + tail.linear_acceleration);
    angvel_avr -= xc.bg;
    acc_avr = acc_avr * g_scale_gravity - xc.ba;
    const double dt = head.time - tail.time;
    const Eigen::Matrix3d Exp_f = math::Exp(angvel_avr, dt);
    acc_imu = R_imu * acc_avr + xc.g;
    pos_imu = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;
    vel_imu = vel_imu + acc_imu * dt;
    R_imu = R_imu * Exp_f;
    const double offt = head.time - pcl_beg_time;
    imu_poses.emplace_back(offt, R_imu, pos_imu, vel_imu, angvel_avr, acc_imu);
  }

  PointVar pv;
  pv.var.setIdentity();
  if (point_notime) {
    for (const sensor::PointType& ap : pl.points) {
      pv.pnt << ap.x, ap.y, ap.z;
      pv.pnt = extrin.R * pv.pnt + extrin.p;
      pvec->push_back(pv);
    }
    return;
  }
  if (pl.empty() || imu_poses.empty()) {
    return;
  }
  auto it_pcl = pl.points.end() - 1;
  for (ImuState& head : imu_poses) {
    R_imu = head.R;
    acc_imu = head.ba;
    vel_imu = head.v;
    pos_imu = head.p;
    angvel_avr = head.bg;
    for (; it_pcl->curvature > head.t; --it_pcl) {
      const double dt = it_pcl->curvature - head.t;
      const Eigen::Matrix3d R_i = R_imu * math::Exp(angvel_avr, dt);
      const Eigen::Vector3d T_ei =
          pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - xc.p;
      const Eigen::Vector3d P_i(it_pcl->x, it_pcl->y, it_pcl->z);
      pv.pnt = xc.R.transpose() * (R_i * (extrin.R * P_i + extrin.p) + T_ei);
      pvec->push_back(pv);
      if (it_pcl == pl.points.begin()) {
        break;
      }
    }
  }
}

int MotionInit(MotionInitInput* input, Eigen::MatrixXd* hess,
               LidarFactor* voxhess, std::vector<ImuState>* x_buf,
               std::unordered_map<VoxelKey, SlidingOctree*>* surf_map,
               std::unordered_map<VoxelKey, SlidingOctree*>* surf_map_slide,
               std::vector<PointVarListPtr>* pvec_buf, int win_size,
               std::vector<std::vector<FrameWindow*>>* sws, ImuState* x_curr,
               std::deque<ImuPreintegration*>* imu_pre_buf) {
  if (input == nullptr || voxhess == nullptr || x_buf == nullptr ||
      surf_map == nullptr || surf_map_slide == nullptr || pvec_buf == nullptr ||
      sws == nullptr || sws->empty() || x_curr == nullptr ||
      imu_pre_buf == nullptr) {
    return 0;
  }
  if (static_cast<int>(input->clouds.size()) < win_size ||
      static_cast<int>(x_buf->size()) < win_size) {
    return 0;
  }

  VectorList<3> pwld;
  int converge_flag = 0;
  const ScopedFrontendVoxelParameters original_parameters;
  min_eigen_value = 0.02;
  for (double& v : plane_eigenvalue_thresholds) {
    v = 1.0 / 4.0;
  }

  double converge_thre = 0.05;
  bool is_degrade = true;
  Eigen::Vector3d eigvalue = Eigen::Vector3d::Zero();

  for (int iterCnt = 0; iterCnt < 10; ++iterCnt) {
    if (converge_flag == 1) {
      min_eigen_value = original_parameters.min_eigen_value;
      plane_eigenvalue_thresholds = original_parameters.plane_thresholds;
    }

    std::vector<SlidingOctree*> octos;
    for (auto& kv : *surf_map) {
      kv.second->tras_ptr(octos);
      kv.second->clear_slwd((*sws)[0]);
      delete kv.second;
    }
    for (SlidingOctree* o : octos) {
      delete o;
    }
    surf_map->clear();
    surf_map_slide->clear();
    octos.clear();

    for (int i = 0; i < win_size; ++i) {
      pwld.clear();
      if (!(*pvec_buf)[static_cast<size_t>(i)]) {
        (*pvec_buf)[static_cast<size_t>(i)] = std::make_shared<PointVarList>();
      }
      (*pvec_buf)[static_cast<size_t>(i)]->clear();
      const int l = i == 0 ? i : i - 1;
      MotionBlur(*input->clouds[static_cast<size_t>(i)],
                 (*pvec_buf)[static_cast<size_t>(i)].get(),
                 (*x_buf)[static_cast<size_t>(i)],
                 (*x_buf)[static_cast<size_t>(l)],
                 input->imus[static_cast<size_t>(i)],
                 input->beg_times[static_cast<size_t>(i)], input->extrin,
                 input->point_notime);

      if (converge_flag == 1) {
        for (PointVar& pv : *(*pvec_buf)[static_cast<size_t>(i)]) {
          CalcBodyVar(pv.pnt, static_cast<float>(input->dept_err),
                      static_cast<float>(input->beam_err), &pv.var);
        }
        UpdateWorldVars((*x_buf)[static_cast<size_t>(i)],
                        (*pvec_buf)[static_cast<size_t>(i)].get(), &pwld);
      } else {
        for (PointVar& pv : *(*pvec_buf)[static_cast<size_t>(i)]) {
          pwld.push_back((*x_buf)[static_cast<size_t>(i)].R * pv.pnt +
                         (*x_buf)[static_cast<size_t>(i)].p);
        }
      }
      CutVoxel(*surf_map, (*pvec_buf)[static_cast<size_t>(i)], i,
               *surf_map_slide, win_size, pwld, (*sws)[0]);
    }

    voxhess->clear();
    voxhess->win_size = win_size;
    for (auto& kv : *surf_map) {
      kv.second->Recut(win_size, *x_buf, (*sws)[0]);
      kv.second->CollectFactors(*voxhess);
    }
    if (voxhess->plvec_voxels.size() < 10) {
      break;
    }

    GravityLiBaOptimizer opt_lsv;
    std::vector<double> resis;
    opt_lsv.DampingIter(*x_buf, *voxhess, *imu_pre_buf, resis, hess, 3);

    for (int i = 0; i < win_size - 1 && !imu_pre_buf->empty(); ++i) {
      delete imu_pre_buf->front();
      imu_pre_buf->pop_front();
    }
    imu_pre_buf->clear();
    for (int i = 1; i < win_size; ++i) {
      imu_pre_buf->push_back(
          new ImuPreintegration((*x_buf)[static_cast<size_t>(i - 1)].bg,
                                (*x_buf)[static_cast<size_t>(i - 1)].ba));
      imu_pre_buf->back()->PushImu(input->imus[static_cast<size_t>(i)]);
    }

    if (resis.size() >= 2 &&
        std::fabs(resis[0] - resis[1]) / std::max(resis[0], 1e-9) <
            converge_thre &&
        iterCnt >= 2) {
      Eigen::Matrix3d nnt = Eigen::Matrix3d::Zero();
      for (Eigen::Matrix3d& evec : voxhess->eig_vectors) {
        const Eigen::Vector3d v3 = evec.col(0);
        nnt += v3 * v3.transpose();
      }
      const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(nnt);
      eigvalue = saes.eigenvalues();
      is_degrade = eigvalue[0] < 15.0;
      converge_thre = 0.01;
      if (converge_flag == 0) {
        AlignGravityStates(x_buf);
        converge_flag = 1;
        continue;
      }
      break;
    }
  }

  *x_curr = (*x_buf)[static_cast<size_t>(win_size - 1)];
  const double gnm = x_curr->g.norm();
  if (is_degrade || gnm < 9.6 || gnm > 10.0) {
    converge_flag = 0;
  }
  if (converge_flag == 0) {
    std::vector<SlidingOctree*> octos;
    for (auto& kv : *surf_map) {
      kv.second->tras_ptr(octos);
      kv.second->clear_slwd((*sws)[0]);
      delete kv.second;
    }
    for (SlidingOctree* o : octos) {
      delete o;
    }
    surf_map->clear();
    surf_map_slide->clear();
  }

  VLOG(1) << "motion_init eig=" << eigvalue.transpose()
          << " flag=" << converge_flag;
  input->clouds.clear();
  input->imus.clear();
  input->beg_times.clear();
  return converge_flag;
}

}  // namespace vxslam
}  // namespace atlas
