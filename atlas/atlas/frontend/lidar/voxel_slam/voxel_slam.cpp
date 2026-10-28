/*
 * Copyright 2026 The Openbot Authors
 */
#include "frontend/lidar/voxel_slam/voxel_slam.hpp"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <chrono>
#include <cmath>

#include "backend/global_mapping/gtsam_pgo.hpp"
#include "backend/loop_closure/lidar/loop_update.hpp"
#include "common/math/so3.hpp"
#include "frontend/lidar/odometry/downsample.hpp"
#include "frontend/lidar/odometry/normal_icp.hpp"
#include "frontend/lidar/odometry/point_cov.hpp"
#include "glog/logging.h"
#include "map/voxel/hierarchical_bundle_adjustment.hpp"
#include "map/voxel/keyframe_load.hpp"
#include "map/voxel/map_ops.hpp"
#include "map/voxel/serialization/voxel_map_serializer.hpp"
#include "optimization/voxel/pose_graph.hpp"

namespace atlas {
namespace vxslam {

Odometry::Odometry() : Odometry(OdometryConfig{}) {}

Odometry::Odometry(const OdometryConfig& config) : config_(config) {}

Odometry::~Odometry() {
  ShutdownGba();
  ClearImuPreintegrations();
  for (auto& kv : feat_map_) {
    delete kv.second;
  }
  feat_map_.clear();
  for (Keyframe* kf : keyframes_) {
    delete kf;
  }
  keyframes_.clear();
  for (PreviousMapSession& s : prev_sessions_) {
    for (Keyframe* kf : s.keyframes) {
      delete kf;
    }
    s.keyframes.clear();
  }
  prev_sessions_.clear();
  for (Keyframe* smp : gba_submaps_) {
    delete smp;
  }
  gba_submaps_.clear();
  if (!mp_.empty() && mp == mp_.data()) {
    mp = nullptr;
  }
}

void Odometry::ShutdownGba() {
  finish_gba_ = true;
  stop_ = true;
  if (gba_thread_.joinable()) {
    gba_thread_.join();
  }
}

void Odometry::ClearImuPreintegrations() {
  for (ImuPreintegration* preintegration : imu_pres_) {
    delete preintegration;
  }
  imu_pres_.clear();
}

bool Odometry::Finalize(std::string* error) {
  ShutdownGba();
  return SaveSession(error);
}

sensor::PointCloudXYZI Odometry::GlobalMapSnapshot() const {
  sensor::PointCloudXYZI map;
  std::lock_guard<std::mutex> lock(map_mutex_);
  const auto append_session = [&map](const std::vector<Keyframe*>& keyframes) {
    for (const Keyframe* keyframe : keyframes) {
      if (keyframe == nullptr || !keyframe->plptr) {
        continue;
      }
      const Eigen::Matrix3d& rotation = keyframe->x0.R;
      const Eigen::Vector3d& translation = keyframe->x0.p;
      for (const PointType& point : keyframe->plptr->points) {
        const Eigen::Vector3d world =
            rotation * Eigen::Vector3d(point.x, point.y, point.z) + translation;
        PointType output = point;
        output.x = static_cast<float>(world.x());
        output.y = static_cast<float>(world.y());
        output.z = static_cast<float>(world.z());
        map.push_back(output);
      }
    }
  };
  for (const PreviousMapSession& session : prev_sessions_) {
    append_session(session.keyframes);
  }
  append_session(keyframes_);
  return map;
}

std::vector<ImuState> Odometry::KeyframePoseSnapshot() const {
  std::vector<ImuState> poses;
  std::lock_guard<std::mutex> lock(map_mutex_);
  poses.reserve(scan_poses_.size());
  for (const ScanPosePtr& scan : scan_poses_) {
    if (scan) {
      poses.push_back(scan->x);
    }
  }
  return poses;
}

void Odometry::SetConfig(const OdometryConfig& config) { config_ = config; }

void Odometry::ApplyConfigGlobals() {
  voxel_size = config_.voxel_size;
  min_eigen_value = config_.min_eigen_value;
  max_layer = config_.max_layer;
  imu_coef = config_.imu_coef;
  min_ba_point = config_.minimum_bundle_adjustment_points;
  plane_eigenvalue_thresholds = config_.plane_eigenvalue_thresholds;
  while (static_cast<int>(plane_eigenvalue_thresholds.size()) <
         config_.max_layer + 1) {
    plane_eigenvalue_thresholds.push_back(1.0);
  }
  for (double& v : plane_eigenvalue_thresholds) {
    if (v > 1e-9) {
      v = 1.0 / v;
    }
  }
  min_point = Eigen::Vector4d(5, 5, 5, 5);
}

void Odometry::Initialize() {
  if (config_.win_size <= 0 || config_.worker_thread_count <= 0 ||
      config_.minimum_bundle_adjustment_points <= 0 ||
      config_.bias_gyro_random_walk < 0.0 ||
      config_.bias_accel_random_walk < 0.0 || config_.gba_voxel_size <= 0.0 ||
      config_.gba_minimum_eigenvalue <= 0.0 ||
      config_.gba_maximum_iteration_count <= 0) {
    initialized_ = false;
    LOG(ERROR) << "Invalid Voxel-SLAM configuration";
    return;
  }
  ApplyConfigGlobals();
  local_mapping_thread_pool_ = std::make_unique<common::ThreadPool>(
      static_cast<std::size_t>(config_.worker_thread_count));
  extrin_.R = config_.extrinsic_R;
  extrin_.p = config_.extrinsic_t;
  imu_ekf_.lid_rot_to_imu = config_.extrinsic_R;
  imu_ekf_.lid_offset_to_imu = config_.extrinsic_t;
  imu_ekf_.cov_gyr = config_.cov_gyr;
  imu_ekf_.cov_acc = config_.cov_acc;
  imu_ekf_.cov_bias_gyr =
      Eigen::Vector3d::Constant(config_.bias_gyro_random_walk);
  imu_ekf_.cov_bias_acc =
      Eigen::Vector3d::Constant(config_.bias_accel_random_walk);
  imu_ekf_.point_notime = config_.point_notime ? 1 : 0;
  imu_ekf_.livox_imu_scale = config_.livox_imu_scale;

  mp_.resize(static_cast<size_t>(config_.win_size));
  for (int i = 0; i < config_.win_size; ++i) {
    mp_[i] = i;
  }
  mp = mp_.data();

  x_buf_.assign(static_cast<size_t>(config_.win_size), ImuState());
  pvec_buf_.assign(static_cast<size_t>(config_.win_size), nullptr);
  sws_.assign(static_cast<size_t>(std::max(1, config_.worker_thread_count)),
              {});
  win_count_ = 0;
  motion_init_flag_ = 1;
  init_buf_ = MotionInitInput{};
  init_buf_.extrin = extrin_;
  init_buf_.dept_err = config_.dept_err;
  init_buf_.beam_err = config_.beam_err;
  init_buf_.point_notime = config_.point_notime;
  x_curr_.SetZero();

  if (config_.enable_triangle) {
    triangle_ = std::make_unique<loop_closure::triangle::TriangleDetector>(
        config_.triangle);
  }
  if (config_.enable_gba) {
    stop_ = false;
    gba_thread_ = std::thread([this] { GbaWorker(); });
  }
  LoadPreviousMaps();
  EnsureSessionCounters();
  initialized_ = true;
  LOG(INFO) << "Odometry initialized (win=" << config_.win_size
            << " voxel=" << config_.voxel_size << ")";
}

void Odometry::LoadPreviousMaps() {
  if (config_.previous_map.empty() || config_.save_path.empty()) {
    return;
  }
  PreviousMapOptions opt;
  opt.save_path = config_.save_path;
  opt.previous_map = config_.previous_map;
  opt.win_size = config_.win_size;
  opt.voxel_size = config_.voxel_size;
  opt.triangle = config_.triangle;
  if (!PreviousMapRead(opt, &prev_sessions_, &lp_edges_)) {
    LOG(WARNING) << "PreviousMapRead returned empty";
    return;
  }
  history_kfsize_ = 0;
  for (const PreviousMapSession& s : prev_sessions_) {
    history_kfsize_ += static_cast<int>(s.keyframes.size());
  }
  cur_session_id_ = static_cast<int>(prev_sessions_.size());
  EnsureSessionCounters();
  LOG(INFO) << "Loaded " << prev_sessions_.size()
            << " previous sessions, history_kf=" << history_kfsize_
            << " cur_session=" << cur_session_id_;
}

void Odometry::EnsureSessionCounters() {
  const int n = cur_session_id_ + 1;
  if (static_cast<int>(jours_.size()) < n) {
    jours_.resize(static_cast<size_t>(n), 0.0);
  }
  if (static_cast<int>(relc_counts_.size()) < n) {
    const int fill = config_.prev_halt;
    relc_counts_.resize(static_cast<size_t>(n), fill);
  }
}

void Odometry::AddLidar(const sensor::PointCloudXYZI::Ptr& cloud,
                        double header_time) {
  if (!cloud) {
    return;
  }
  sensor::PointCloudXYZI filtered;
  ProcessLidarVendor(*cloud, &filtered, config_.lidar);
  auto ptr = std::make_shared<sensor::PointCloudXYZI>(std::move(filtered));
  sync_.PushLidar(ptr, header_time);
}

void Odometry::AddImu(const sensor::ImuData& imu) { sync_.PushImu(imu); }

bool Odometry::LioStateEstimation(PointVarListPtr pvec) {
  if (!pvec || pvec->empty()) {
    return false;
  }
  const ImuState x_prop = x_curr_;
  constexpr int kMaxIter = 4;
  Eigen::Matrix<double, kStateDim, kStateDim> G =
      Eigen::Matrix<double, kStateDim, kStateDim>::Zero();
  Eigen::Matrix<double, kStateDim, kStateDim> H_T_H =
      Eigen::Matrix<double, kStateDim, kStateDim>::Zero();
  const Eigen::Matrix<double, kStateDim, kStateDim> I_STATE =
      Eigen::Matrix<double, kStateDim, kStateDim>::Identity();
  int rematch_num = 0;
  Eigen::Matrix3d nnt = Eigen::Matrix3d::Zero();
  const Eigen::Matrix<double, kStateDim, kStateDim> cov_inv =
      x_curr_.cov.inverse();
  const int psize = static_cast<int>(pvec->size());
  std::vector<SlidingOctree*> octos(static_cast<size_t>(psize), nullptr);

  for (int iterCount = 0; iterCount < kMaxIter; ++iterCount) {
    Eigen::Matrix<double, 6, 6> HTH = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> HTz = Eigen::Matrix<double, 6, 1>::Zero();
    const Eigen::Matrix3d rot_var = x_curr_.cov.block<3, 3>(0, 0);
    const Eigen::Matrix3d tsl_var = x_curr_.cov.block<3, 3>(3, 3);
    int match_num = 0;
    nnt.setZero();

    for (int i = 0; i < psize; ++i) {
      PointVar& pv = (*pvec)[static_cast<size_t>(i)];
      const Eigen::Matrix3d phat = math::SkewSymmetric(pv.pnt);
      Eigen::Matrix3d var_world = x_curr_.R * pv.var * x_curr_.R.transpose() +
                                  phat * rot_var * phat.transpose() + tsl_var;
      Eigen::Vector3d wld = x_curr_.R * pv.pnt + x_curr_.p;
      double sigma_d = 0;
      Plane* pla = nullptr;
      int flag = 0;
      if (octos[static_cast<size_t>(i)] != nullptr &&
          octos[static_cast<size_t>(i)]->inside(wld)) {
        double max_prob = 0;
        flag = octos[static_cast<size_t>(i)]->Match(
            wld, pla, max_prob, var_world, sigma_d,
            octos[static_cast<size_t>(i)]);
      } else {
        flag = Match(feat_map_, wld, pla, var_world, sigma_d,
                     octos[static_cast<size_t>(i)]);
      }
      if (flag && pla != nullptr) {
        const double R_inv = 1.0 / (0.0005 + sigma_d);
        const double resi = pla->normal.dot(wld - pla->center);
        Eigen::Matrix<double, 6, 1> jac;
        jac.head(3) = phat * x_curr_.R.transpose() * pla->normal;
        jac.tail(3) = pla->normal;
        HTH += R_inv * jac * jac.transpose();
        HTz -= R_inv * jac * resi;
        nnt += pla->normal * pla->normal.transpose();
        ++match_num;
      }
    }

    H_T_H.block<6, 6>(0, 0) = HTH;
    const Eigen::Matrix<double, kStateDim, kStateDim> K_1 =
        (H_T_H + cov_inv).inverse();
    G.block<kStateDim, 6>(0, 0) = K_1.block<kStateDim, 6>(0, 0) * HTH;
    const Eigen::Matrix<double, kStateDim, 1> vec = x_prop - x_curr_;
    Eigen::Matrix<double, kStateDim, 1> solution =
        K_1.block<kStateDim, 6>(0, 0) * HTz + vec -
        G.block<kStateDim, 6>(0, 0) * vec.block<6, 1>(0, 0);
    x_curr_ += solution;
    const Eigen::Vector3d rot_add = solution.block<3, 1>(0, 0);
    const Eigen::Vector3d tra_add = solution.block<3, 1>(3, 0);

    bool flg_converged =
        (rot_add.norm() * 57.3 < 0.01) && (tra_add.norm() * 100 < 0.015);
    if (flg_converged || ((rematch_num == 0) && (iterCount == kMaxIter - 2))) {
      ++rematch_num;
    }
    if (rematch_num >= 2 || iterCount == kMaxIter - 1) {
      x_curr_.cov = (I_STATE - G) * x_curr_.cov;
      break;
    }
    (void)match_num;
  }

  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(nnt);
  if (saes.eigenvalues()[0] < 14.0) {
    ++degrade_cnt_;
    return false;
  }
  degrade_cnt_ = 0;
  return true;
}

void Odometry::LocalBaIfReady() {
  if (!config_.enable_local_ba || win_count_ < config_.win_size) {
    return;
  }
  auto local_factors = std::make_unique<LidarFactor>(config_.win_size);
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    RecutSlidingVoxelMap(&feat_map_, win_count_, &x_buf_, local_factors.get(),
                         &sws_, local_mapping_thread_pool_.get(),
                         config_.worker_thread_count);
    sw_pool_ = sws_[0];
  }
  if (local_factors->plvec_voxels.empty()) {
    return;
  }
  Eigen::MatrixXd hess;
  // Voxel-SLAM: g_update==2 forces Gravity LI-BA after cross-session loop.
  if (g_update_ == 2 || config_.enable_gravity_ba) {
    GravityLiBaOptimizer opt;
    std::vector<double> resis;
    opt.DampingIter(x_buf_, *local_factors, imu_pres_, resis, &hess, 5);
    if (g_update_ == 2) {
      g_update_ = 0;
      LOG(INFO) << "g_update Gravity LI-BA g.norm=" << x_buf_[0].g.norm();
    }
  } else {
    LiBaOptimizer opt;
    opt.DampingIter(x_buf_, *local_factors, imu_pres_, &hess);
  }
  optimized_local_factors_ = std::move(local_factors);
  x_curr_ = x_buf_[static_cast<size_t>(config_.win_size - 1)];
  // ScanPose odom edge noise from Hessian (Voxel-SLAM bl->v6).
  if (hess.rows() >= DIM + 6 && hess.cols() >= DIM + 6) {
    last_scan_v6_ = hess.block<6, 6>(0, DIM).diagonal();
    for (int i = 0; i < 6; ++i) {
      last_scan_v6_[i] = 1.0 / std::max(1e-12, std::fabs(last_scan_v6_[i]));
    }
  }
}

bool Odometry::TrySessionLoops(
    const sensor::PointCloudXYZI& plane_src,
    const std::vector<loop_closure::triangle::TriangleDescriptor>& stds,
    int query_id, const ImuState& xc, double smp_jour) {
  if (stds.empty() || plane_src.empty()) {
    return false;
  }
  std::lock_guard<std::mutex> lock(map_mutex_);
  EnsureSessionCounters();
  Eigen::Matrix<double, 6, 1> v6_init;
  v6_init << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4;

  // Connected sessions before any new edge this frame (Voxel-SLAM ids).
  std::vector<int> connected;
  lp_edges_.Connect(cur_session_id_, &connected);
  if (connected.empty()) {
    connected.push_back(cur_session_id_);
  }

  bool is_opt = false;

  auto try_det = [&](int session_id,
                     loop_closure::triangle::TriangleDetector* det,
                     double jud) {
    if (det == nullptr) {
      return;
    }
    auto res = det->SearchLoop(stds, plane_src, query_id);
    if (!res.ok() || res.score <= jud) {
      return;
    }
    const sensor::PointCloudXYZI* plane_tar =
        det->PlaneCloud(res.match_db_index);
    if (plane_tar == nullptr || plane_tar->empty()) {
      return;
    }
    auto tf = res.transform;
    if (!NormalIcp(plane_src, *plane_tar, &tf, config_.icp_eigval)) {
      return;
    }

    ImuState xx;
    bool found_pose = false;
    const auto load_scan = [&](const std::vector<ScanPosePtr>& scans) {
      if (res.match_frame_id >= 0 &&
          res.match_frame_id < static_cast<int>(scans.size()) &&
          scans[static_cast<size_t>(res.match_frame_id)]) {
        xx = scans[static_cast<size_t>(res.match_frame_id)]->x;
        found_pose = true;
      }
    };
    if (session_id == cur_session_id_) {
      load_scan(scan_poses_);
    } else if (session_id >= 0 &&
               session_id < static_cast<int>(prev_sessions_.size())) {
      load_scan(prev_sessions_[static_cast<size_t>(session_id)].scan_poses);
    }
    if (!found_pose) {
      for (const Keyframe* k : keyframes_) {
        if (k != nullptr && k->id == res.match_frame_id) {
          xx = k->x0;
          found_pose = true;
          break;
        }
      }
    }
    if (!found_pose) {
      return;
    }
    const double drift_p = (xx.R * tf.first + xx.p - xc.p).norm();

    bool is_push = false;
    if (session_id == cur_session_id_) {
      double match_jour = 0.0;
      for (const Keyframe* k : keyframes_) {
        if (k != nullptr && k->id == res.match_frame_id) {
          match_jour = k->jour;
          break;
        }
      }
      const double span = std::max(1e-3, smp_jour - match_jour);
      LOG(INFO) << "Loop same-session id=" << res.match_frame_id
                << " score=" << res.score << " drift=" << drift_p
                << " span=" << span;
      if (drift_p / span < config_.ratio_drift) {
        is_push = true;
        if (relc_counts_[static_cast<size_t>(session_id)] > config_.curr_halt &&
            drift_p > 0.10) {
          is_opt = true;
          for (int& cnt : relc_counts_) {
            cnt = 0;
          }
        }
      }
    } else {
      const bool already = std::find(connected.begin(), connected.end(),
                                     session_id) != connected.end();
      const double jour_id = (session_id < static_cast<int>(jours_.size()))
                                 ? jours_[static_cast<size_t>(session_id)]
                                 : 1e-6;
      LOG(INFO) << "Loop cross-session s=" << session_id
                << " match=" << res.match_frame_id << " score=" << res.score
                << " drift=" << drift_p << " jour=" << jour_id;
      if (!already) {
        // First edge to this previous session → always ISAM2.
        is_opt = true;
        relc_counts_[static_cast<size_t>(session_id)] = 0;
        g_update_ = 1;
        is_push = true;
        jours_[static_cast<size_t>(session_id)] = 0.0;
        connected.push_back(session_id);
      } else if (drift_p / std::max(jour_id, 1e-9) < 0.05) {
        jours_[static_cast<size_t>(session_id)] = 1e-6;
        is_push = true;
        if (relc_counts_[static_cast<size_t>(session_id)] > config_.prev_halt &&
            drift_p > 0.25) {
          is_opt = true;
          for (int& cnt : relc_counts_) {
            cnt = 0;
          }
        }
      }
    }

    if (!is_push) {
      return;
    }
    lp_edges_.Push(session_id, cur_session_id_, res.match_frame_id, query_id,
                   tf.second, tf.first, v6_init);
    if (config_.enable_loop) {
      backend::LoopFactor f;
      f.from_id = res.match_frame_id;
      f.to_id = query_id;
      f.T_i_j = transform::Rigid3d(tf.first, Eigen::Quaterniond(tf.second));
      f.information = 1.0 / std::max(v6_init.mean(), 1e-9);
      pose_graph_.AddLoopFactor(f);
    }
  };

  for (size_t i = 0; i < prev_sessions_.size(); ++i) {
    try_det(static_cast<int>(i), prev_sessions_[i].triangle.get(),
            prev_sessions_[i].jud);
  }
  try_det(cur_session_id_, triangle_.get(), config_.jud_default);

  for (int& it : relc_counts_) {
    ++it;
  }
  return is_opt;
}

void Odometry::RunOnlineLoopIsam2() {
  std::lock_guard<std::mutex> lock(map_mutex_);
  if (scan_poses_.empty()) {
    return;
  }
  const ImuState x1 = scan_poses_.back()->x;

  std::vector<std::vector<ScanPosePtr>*> multimap_scans;
  std::vector<std::vector<Keyframe*>*> multimap_kfs;
  for (PreviousMapSession& s : prev_sessions_) {
    multimap_scans.push_back(&s.scan_poses);
    multimap_kfs.push_back(&s.keyframes);
  }
  multimap_scans.push_back(&scan_poses_);
  multimap_kfs.push_back(&keyframes_);

  // Online isOpt: lp edges only (no GBA), same as build_graph(..., 1).
  if (!OptimizeIsam2(&multimap_scans, &multimap_kfs, lp_edges_, gba_edges1_,
                     gba_edges2_, cur_session_id_, /*inject_gba=*/false)) {
    return;
  }

  const ImuState x3 = scan_poses_.back()->x;
  ImuState dx;
  dx.SetZero();
  dx.p = x3.p - x3.R * x1.R.transpose() * x1.p;
  dx.R = x3.R * x1.R.transpose();
  x_key_ = x3;
  x_key_init_ = true;

  history_kfsize_ = 0;
  constexpr int kInitNum = 5;
  const int subsize = static_cast<int>(keyframes_.size());

  {
    for (int i = 0; i < win_count_; ++i) {
      ImuState& x = x_buf_[static_cast<size_t>(i)];
      x.v = dx.R * x.v;
      x.p = dx.R * x.p + dx.p;
      x.R = dx.R * x.R;
      if (g_update_ == 1) {
        x.g = dx.R * x.g;
      }
    }
    if (win_count_ > 0) {
      x_curr_.R = x_buf_[static_cast<size_t>(win_count_ - 1)].R;
      x_curr_.p = x_buf_[static_cast<size_t>(win_count_ - 1)].p;
      x_curr_.g = x_buf_[static_cast<size_t>(win_count_ - 1)].g;
    } else {
      x_curr_.R = x3.R;
      x_curr_.p = x3.p;
    }
    x_curr_.v = dx.R * x_curr_.v;

    // Rebuild local map from corrected window (Voxel-SLAM loop_update).
    LoopUpdateMap(&feat_map_, x_buf_, pvec_buf_, win_count_, config_.win_size,
                  &sw_pool_);

    // Re-inject last keyframes into map; older stay available via loading.
    for (int i = subsize - kInitNum; i < subsize; ++i) {
      if (i < 0) {
        continue;
      }
      Keyframe* sp = keyframes_[static_cast<size_t>(i)];
      if (sp == nullptr || sp->plptr == nullptr) {
        continue;
      }
      sp->exist = 0;
      PointVarList pvec;
      pvec.reserve(sp->plptr->size());
      PointVar pv;
      pv.var.setZero();
      for (const PointType& ap : sp->plptr->points) {
        pv.pnt << ap.x, ap.y, ap.z;
        pv.pnt = sp->x0.R * pv.pnt + sp->x0.p;
        pv.var(0, 0) = ap.normal_x;
        pv.var(1, 1) = ap.normal_y;
        pv.var(2, 2) = ap.normal_z;
        pvec.push_back(pv);
      }
      CutVoxel(feat_map_, pvec, config_.win_size, jour_);
    }
    if (subsize > kInitNum) {
      for (int i = 0; i < subsize - kInitNum; ++i) {
        if (keyframes_[static_cast<size_t>(i)] != nullptr) {
          keyframes_[static_cast<size_t>(i)]->exist = 1;
        }
      }
      history_kfsize_ = subsize - kInitNum;
    }
    for (PreviousMapSession& s : prev_sessions_) {
      for (Keyframe* kf : s.keyframes) {
        if (kf != nullptr) {
          kf->exist = 1;
          ++history_kfsize_;
        }
      }
    }
    jour_ = 0.0;
    if (g_update_ == 1) {
      g_update_ = 2;
    }
  }
  LOG(INFO) << "Online ISAM2 applied dx.p.norm=" << dx.p.norm()
            << " history_kf=" << history_kfsize_;
}

void Odometry::SubmitLoopKeyframe(const sensor::PointCloudXYZI& cloud) {
  EnsureSessionCounters();
  ++keyframe_id_;
  // Session-local index for GTSAM stepsizes / BTC frame_number.
  const int local_id = static_cast<int>(scan_poses_.size());

  auto sp = std::make_shared<ScanPose>(x_curr_, nullptr);
  sp->v6 = last_scan_v6_;
  if (!x_key_init_) {
    x_key_ = x_curr_;
    x_key_init_ = true;
  }
  const double len = (x_curr_.p - x_key_.p).norm();
  for (double& j : jours_) {
    j += len;
  }
  x_key_ = x_curr_;
  session_jour_ = jours_[static_cast<size_t>(cur_session_id_)];

  ImuState xc = x_curr_;
  auto* smp = new Keyframe(xc);
  smp->id = local_id;
  smp->mp = cur_session_id_;
  smp->exist = 1;
  smp->jour = static_cast<float>(session_jour_);
  *smp->plptr = cloud;
  DownsampleVoxel(smp->plptr.get(), config_.voxel_size / 10.0);

  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    scan_poses_.push_back(sp);
    keyframes_.push_back(smp);
    gba_poses_.push_back(x_curr_);
    gba_clouds_.push_back(
        std::make_shared<const sensor::PointCloudXYZI>(cloud));
    gba_local_ids_.push_back(static_cast<int>(keyframes_.size()) - 1);
  }

  if (triangle_) {
    std::vector<loop_closure::triangle::TriangleDescriptor> stds;
    sensor::PointCloudXYZI plane_src;
    triangle_->GenerateDescriptors(cloud, local_id, &stds, &plane_src);
    const bool is_opt =
        TrySessionLoops(plane_src, stds, local_id, x_curr_, session_jour_);
    triangle_->AddDescriptors(stds, plane_src);
    if (is_opt) {
      RunOnlineLoopIsam2();
    }
  }
}

void Odometry::SystemReset() {
  LOG(WARNING) << "Odometry system_reset (degrade)";
  std::lock_guard<std::mutex> lock(map_mutex_);
  ClearImuPreintegrations();
  for (auto& kv : feat_map_) {
    delete kv.second;
  }
  feat_map_.clear();
  optimized_local_factors_.reset();
  win_count_ = 0;
  degrade_cnt_ = 0;
  motion_init_flag_ = 1;
  init_buf_ = MotionInitInput{};
  init_buf_.extrin = extrin_;
  init_buf_.dept_err = config_.dept_err;
  init_buf_.beam_err = config_.beam_err;
  init_buf_.point_notime = config_.point_notime;
  imu_ekf_.init_flag = false;
  imu_ekf_.init_num = 0;
  system_ready_ = false;
  kd_odom_.Clear();
  x_curr_.SetZero();
}

void Odometry::RunTopDownPgo() {
  bool optimized = false;
  {
    // Hold the map lock while graph nodes are flattened and optimized. The
    // frontend may append a keyframe concurrently with the GBA worker.
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (keyframes_.empty() && scan_poses_.empty()) {
      return;
    }

    std::vector<std::vector<ScanPosePtr>*> multimap_scans;
    std::vector<std::vector<Keyframe*>*> multimap_kfs;
    for (PreviousMapSession& session : prev_sessions_) {
      multimap_scans.push_back(&session.scan_poses);
      multimap_kfs.push_back(&session.keyframes);
    }
    multimap_scans.push_back(&scan_poses_);
    multimap_kfs.push_back(&keyframes_);
    ImuState previous_current;
    const bool has_previous_current =
        !scan_poses_.empty() && scan_poses_.back();
    if (has_previous_current) {
      previous_current = scan_poses_.back()->x;
    }

    if (TopDownIsam2(&multimap_scans, &multimap_kfs, lp_edges_, gba_edges1_,
                     gba_edges2_, cur_session_id_)) {
      if (has_previous_current && !scan_poses_.empty() && scan_poses_.back()) {
        const ImuState& corrected = scan_poses_.back()->x;
        const Eigen::Matrix3d delta_rotation =
            corrected.R * previous_current.R.transpose();
        const Eigen::Vector3d delta_translation =
            corrected.p - delta_rotation * previous_current.p;
        for (int i = 0; i < win_count_; ++i) {
          ImuState& state = x_buf_[static_cast<size_t>(i)];
          state.R = delta_rotation * state.R;
          state.p = delta_rotation * state.p + delta_translation;
          state.v = delta_rotation * state.v;
        }
        x_curr_.R = delta_rotation * x_curr_.R;
        x_curr_.p = delta_rotation * x_curr_.p + delta_translation;
        x_curr_.v = delta_rotation * x_curr_.v;
        x_key_ = scan_poses_.back()->x;
      }
      optimized = true;
    } else {
      // The solver fallback uses flattened ids across all sessions,
      // matching GTSAM's stepsizes mapping.
      pose_graph_.Clear();
      std::vector<int> ids(multimap_scans.size());
      std::vector<int> stepsizes(multimap_scans.size() + 1, 0);
      for (size_t i = 0; i < ids.size(); ++i) {
        ids[i] = static_cast<int>(i);
        stepsizes[i + 1] =
            stepsizes[i] + static_cast<int>(multimap_scans[i]->size());
      }
      for (size_t session = 0; session < multimap_scans.size(); ++session) {
        const auto& scans = *multimap_scans[session];
        for (size_t i = 0; i < scans.size(); ++i) {
          if (!scans[i]) {
            continue;
          }
          const int node = stepsizes[session] + static_cast<int>(i);
          pose_graph_.AddNode(
              node, transform::Rigid3d(scans[i]->x.p,
                                       Eigen::Quaterniond(scans[i]->x.R)));
          if (i == 0 || !scans[i - 1]) {
            continue;
          }
          const ImuState& from = scans[i - 1]->x;
          const ImuState& to = scans[i]->x;
          const Eigen::Matrix3d relative_rotation = from.R.transpose() * to.R;
          const Eigen::Vector3d relative_translation =
              from.R.transpose() * (to.p - from.p);
          const double variance =
              std::max(1e-9, scans[i - 1]->v6.cwiseAbs().mean());
          pose_graph_.AddOdometryEdge(
              node - 1, node,
              transform::Rigid3d(relative_translation,
                                 Eigen::Quaterniond(relative_rotation)),
              1.0 / variance);
        }
      }
      HierarchicalBundleAdjustment::TopDownApplyEdges(
          &pose_graph_, gba_edges1_, gba_edges2_, ids, stepsizes);
      const auto add_edges = [&](const PgoEdges& batch) {
        for (const PgoEdge& edge : batch.edges) {
          std::vector<int> session_indices(2, -1);
          if (!edge.IsAdapt(ids, &session_indices) ||
              edge.ids1.size() != edge.rots.size() ||
              edge.ids2.size() != edge.rots.size() ||
              edge.tras.size() != edge.rots.size() ||
              edge.covs.size() != edge.rots.size()) {
            continue;
          }
          const int from_base =
              stepsizes[static_cast<size_t>(session_indices[0])];
          const int to_base =
              stepsizes[static_cast<size_t>(session_indices[1])];
          for (size_t i = 0; i < edge.rots.size(); ++i) {
            if (edge.ids1[i] < 0 || edge.ids2[i] < 0 ||
                !std::isfinite(edge.covs[i].mean())) {
              continue;
            }
            backend::LoopFactor factor;
            factor.from_id = from_base + edge.ids1[i];
            factor.to_id = to_base + edge.ids2[i];
            factor.T_i_j = transform::Rigid3d(edge.tras[i],
                                              Eigen::Quaterniond(edge.rots[i]));
            factor.information =
                1.0 / std::max(edge.covs[i].cwiseAbs().mean(), 1e-9);
            pose_graph_.AddLoopFactor(factor);
          }
        }
      };
      add_edges(lp_edges_);
      const auto result = pose_graph_.Optimize(10);
      LOG(INFO) << "TopDown PoseGraph fallback ok=" << result.ok
                << " root_mean_cost=" << result.root_mean_cost;
      if (result.ok) {
        for (size_t session = 0; session < multimap_scans.size(); ++session) {
          auto& scans = *multimap_scans[session];
          auto& keyframes = *multimap_kfs[session];
          for (size_t i = 0; i < scans.size(); ++i) {
            const int node = stepsizes[session] + static_cast<int>(i);
            if (!scans[i] || !pose_graph_.HasNode(node)) {
              continue;
            }
            const transform::Rigid3d pose = pose_graph_.Pose(node);
            scans[i]->SetState(pose.rotation().toRotationMatrix(),
                               pose.translation());
            if (i < keyframes.size() && keyframes[i]) {
              keyframes[i]->x0.R = pose.rotation().toRotationMatrix();
              keyframes[i]->x0.p = pose.translation();
            }
          }
        }
        if (has_previous_current && !scan_poses_.empty() &&
            scan_poses_.back()) {
          const ImuState& corrected = scan_poses_.back()->x;
          const Eigen::Matrix3d delta_rotation =
              corrected.R * previous_current.R.transpose();
          const Eigen::Vector3d delta_translation =
              corrected.p - delta_rotation * previous_current.p;
          for (int i = 0; i < win_count_; ++i) {
            ImuState& state = x_buf_[static_cast<size_t>(i)];
            state.R = delta_rotation * state.R;
            state.p = delta_rotation * state.p + delta_translation;
            state.v = delta_rotation * state.v;
          }
          x_curr_.R = delta_rotation * x_curr_.R;
          x_curr_.p = delta_rotation * x_curr_.p + delta_translation;
          x_curr_.v = delta_rotation * x_curr_.v;
          x_key_ = scan_poses_.back()->x;
        }
        optimized = true;
      }
    }
  }
  if (optimized) {
    ApplyLoopUpdate();
    SyncStateMirror();
  }
}

void Odometry::GbaWorker() {
  const int local_window_size = std::max(2, config_.gba_local_win);
  const int merge_window_size = std::max(1, config_.gba_merge_size);
  GlobalBundleAdjustmentConfiguration global_bundle_adjustment_configuration;
  global_bundle_adjustment_configuration.voxel_size = config_.gba_voxel_size;
  global_bundle_adjustment_configuration.minimum_eigenvalue =
      config_.gba_minimum_eigenvalue;
  global_bundle_adjustment_configuration.eigenvalue_thresholds =
      config_.gba_eigenvalue_thresholds.empty()
          ? config_.plane_eigenvalue_thresholds
          : config_.gba_eigenvalue_thresholds;
  global_bundle_adjustment_configuration.maximum_octree_layer =
      config_.max_layer;
  global_bundle_adjustment_configuration.maximum_iteration_count =
      config_.gba_maximum_iteration_count;
  global_bundle_adjustment_configuration.worker_thread_count =
      config_.worker_thread_count;
  HierarchicalBundleAdjustment hierarchical_optimizer(
      global_bundle_adjustment_configuration);

  while (!stop_ || finish_gba_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Bottom-up local window → gba_edges1 + submap.
    std::vector<ImuState> xs;
    std::vector<Keyframe*> smp_local;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      if (static_cast<int>(gba_local_ids_.size()) < local_window_size &&
          !finish_gba_) {
        continue;
      }
      if (gba_local_ids_.empty()) {
        if (finish_gba_) {
          // fall through to top-down
        } else {
          continue;
        }
      } else {
        for (int idx : gba_local_ids_) {
          if (idx < 0 || idx >= static_cast<int>(keyframes_.size())) {
            continue;
          }
          Keyframe* smp = keyframes_[static_cast<size_t>(idx)];
          if (smp == nullptr) {
            continue;
          }
          xs.push_back(smp->x0);
          smp_local.push_back(smp);
        }
      }
    }

    if (!smp_local.empty()) {
      std::vector<int> maps = {cur_session_id_};
      hierarchical_optimizer.AddEdgesFromHierarchicalBundleAdjustment(
          &xs, smp_local, &gba_edges1_, maps,
          /*maximum_iteration_count=*/1,
          std::min(2,
                   global_bundle_adjustment_configuration.worker_thread_count));
      // Create submap keyframe (merged cloud of first pose).
      ImuState x0 = xs.front();
      auto* gba_smp = new Keyframe(x0);
      gba_smp->id = smp_local.front()->id;
      gba_smp->mp = cur_session_id_;
      if (smp_local.front()->plptr) {
        *gba_smp->plptr = *smp_local.front()->plptr;
      }
      {
        std::lock_guard<std::mutex> lock(map_mutex_);
        gba_submaps_.push_back(gba_smp);
        for (int i = 0; i < merge_window_size && !gba_local_ids_.empty(); ++i) {
          gba_local_ids_.pop_front();
        }
      }
      // Write refined local poses back.
      std::lock_guard<std::mutex> lock(map_mutex_);
      for (size_t i = 0; i < smp_local.size() && i < xs.size(); ++i) {
        smp_local[i]->x0 = xs[i];
      }
    }

    if (!finish_gba_) {
      continue;
    }

    // Level-2 HBA on submaps → gba_edges2, then top-down PGO.
    std::vector<ImuState> sub_xs;
    std::vector<Keyframe*> sub_smps;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      sub_smps = gba_submaps_;
      for (Keyframe* s : sub_smps) {
        if (s != nullptr) {
          sub_xs.push_back(s->x0);
        }
      }
      gba_edges2_.edges.clear();
      gba_edges2_.mates.clear();
    }
    if (sub_smps.size() >= 2) {
      std::vector<int> cnct = {cur_session_id_};
      for (size_t i = 0; i < prev_sessions_.size(); ++i) {
        cnct.push_back(static_cast<int>(i));
      }
      hierarchical_optimizer.AddEdgesFromHierarchicalBundleAdjustment(
          &sub_xs, sub_smps, &gba_edges2_, cnct,
          global_bundle_adjustment_configuration.maximum_iteration_count,
          global_bundle_adjustment_configuration.worker_thread_count);
      std::lock_guard<std::mutex> lock(map_mutex_);
      for (size_t i = 0; i < sub_smps.size() && i < sub_xs.size(); ++i) {
        sub_smps[i]->x0 = sub_xs[i];
      }
    }

    // Full-volume Optimize as residual polish.
    std::vector<ImuState> poses;
    std::vector<std::shared_ptr<const sensor::PointCloudXYZI>> clouds;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      poses = gba_poses_;
      clouds = gba_clouds_;
    }
    if (poses.size() >= 3 && poses.size() == clouds.size()) {
      const double r = hierarchical_optimizer.Optimize(&poses, clouds);
      LOG(INFO) << "HierarchicalBundleAdjustment residual=" << r
                << " poses=" << poses.size();
      std::lock_guard<std::mutex> lock(map_mutex_);
      gba_poses_ = poses;
    }

    RunTopDownPgo();
    finish_gba_ = false;
    LOG(INFO) << "GBA finish: edges1=" << gba_edges1_.edges.size()
              << " edges2=" << gba_edges2_.edges.size()
              << " submaps=" << gba_submaps_.size();
  }
}

void Odometry::RequestFinishGba() { finish_gba_ = true; }

void Odometry::ApplyLoopUpdate() {
  std::lock_guard<std::mutex> lock(map_mutex_);
  LoopUpdateMap(&feat_map_, x_buf_, pvec_buf_, win_count_, config_.win_size,
                &sw_pool_);
  jour_ = 0.0;
}

void Odometry::SyncStateMirror() {
  state_mirror_.rot_end = x_curr_.R;
  state_mirror_.pos_end = x_curr_.p;
  state_mirror_.vel_end = x_curr_.v;
  state_mirror_.bias_g = x_curr_.bg;
  state_mirror_.bias_a = x_curr_.ba;
  state_mirror_.gravity = x_curr_.g;
}

const frontend::StatesGroup& Odometry::state() const { return state_mirror_; }

bool Odometry::TryMotionInit(::atlas::odometry::OdometryOutput* out) {
  if (static_cast<int>(init_buf_.clouds.size()) < config_.win_size) {
    out->status = ::atlas::odometry::ProcessStatus::kPropagateOnly;
    out->soft_skip = true;
    SyncStateMirror();
    return true;
  }
  auto initialization_factors = std::make_unique<LidarFactor>(config_.win_size);
  Eigen::MatrixXd hess;
  std::unordered_map<VoxelKey, SlidingOctree*> slide;
  const int ok = MotionInit(&init_buf_, &hess, initialization_factors.get(),
                            &x_buf_, &feat_map_, &slide, &pvec_buf_,
                            config_.win_size, &sws_, &x_curr_, &imu_pres_);
  if (ok) {
    optimized_local_factors_ = std::move(initialization_factors);
    motion_init_flag_ = 0;
    system_ready_ = true;
    win_count_ = config_.win_size;
    LOG(INFO) << "MotionInit success";
  } else {
    SystemReset();
    LOG(WARNING) << "MotionInit failed / degrade";
  }
  out->status = ::atlas::odometry::ProcessStatus::kPropagateOnly;
  out->soft_skip = true;
  SyncStateMirror();
  return true;
}

bool Odometry::Process(::atlas::odometry::OdometryOutput* out) {
  if (!initialized_ || out == nullptr) {
    return false;
  }
  sync_.SetSlamMode(frontend::kOnlyLio);
  frontend::LidarMeasureGroup meas;
  if (!sync_.Sync(meas)) {
    out->status = ::atlas::odometry::ProcessStatus::kNoPackage;
    return false;
  }
  if (!meas.lidar || meas.lidar->empty()) {
    out->status = ::atlas::odometry::ProcessStatus::kEmptyLidar;
    out->soft_skip = true;
    return true;
  }

  sensor::PointCloudXYZI pl = *meas.lidar;
  // Atlas stores per-point relative time in milliseconds, while the original
  // Voxel-SLAM deskew and motion-initialization equations consume seconds.
  // SensorSync deliberately keeps the shared representation unchanged, so
  // normalize the private working copy at the Voxel-SLAM boundary.
  for (sensor::PointType& point : pl.points) {
    point.curvature *= 1e-3f;
  }
  DownsampleVoxel(&pl, config_.down_size);

  std::deque<sensor::ImuData> imus;
  for (const auto& mg : meas.measures) {
    imus.insert(imus.end(), mg.imu.begin(), mg.imu.end());
  }
  imu_ekf_.pcl_beg_time = meas.lidar_frame_beg_time;
  imu_ekf_.pcl_end_time = meas.lidar_frame_end_time > 0
                              ? meas.lidar_frame_end_time
                              : meas.lidar_frame_beg_time;
  const int ekf_ret = imu_ekf_.Process(&x_curr_, &pl, &imus);
  if (ekf_ret == 0) {
    out->status = ::atlas::odometry::ProcessStatus::kPropagateOnly;
    out->soft_skip = true;
    SyncStateMirror();
    return true;
  }

  if (motion_init_flag_) {
    init_buf_.clouds.push_back(std::make_shared<sensor::PointCloudXYZI>(pl));
    init_buf_.imus.push_back(imus);
    init_buf_.beg_times.push_back(meas.lidar_frame_beg_time);
    const size_t idx = init_buf_.clouds.size() - 1;
    if (idx < x_buf_.size()) {
      x_buf_[idx] = x_curr_;
    }
    if (idx > 0 && idx <= imu_pres_.size() + 1) {
      auto* pre = new ImuPreintegration(x_buf_[idx - 1].bg, x_buf_[idx - 1].ba);
      pre->PushImu(imus);
      imu_pres_.push_back(pre);
    }
    return TryMotionInit(out);
  }

  system_ready_ = true;
  auto pvec = std::make_shared<PointVarList>();
  InitPointVars(extrin_, pl, config_.dept_err, config_.beam_err, pvec.get());

  bool ok = true;
  if (feat_map_.empty()) {
    // Bootstrap like Voxel-SLAM lio_state_estimation_kdtree.
    if (kd_odom_.SeedOrReady(*pvec, x_curr_)) {
      ok = kd_odom_.Estimate(pvec, &x_curr_);
    }
  } else {
    ok = LioStateEstimation(pvec);
  }
  if (!ok && degrade_cnt_ > config_.degrade_bound) {
    SystemReset();
    kd_odom_.Clear();
    out->status = ::atlas::odometry::ProcessStatus::kPropagateOnly;
    out->soft_skip = true;
    return true;
  }

  VectorList<3> pwld;
  UpdateWorldVars(x_curr_, pvec.get(), &pwld);

  if (win_count_ >= config_.win_size) {
    if (!imu_pres_.empty()) {
      delete imu_pres_.front();
      imu_pres_.pop_front();
    }
    for (int i = 0; i < config_.win_size - 1; ++i) {
      x_buf_[static_cast<size_t>(i)] = x_buf_[static_cast<size_t>(i + 1)];
      pvec_buf_[static_cast<size_t>(i)] = pvec_buf_[static_cast<size_t>(i + 1)];
      mp_[i] = i;
    }
    mp_[config_.win_size - 1] = config_.win_size - 1;
    win_count_ = config_.win_size - 1;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      if (!optimized_local_factors_) {
        optimized_local_factors_ =
            std::make_unique<LidarFactor>(config_.win_size);
      }
      MarginalizeSlidingVoxelMap(&feat_map_, jour_, win_count_ + 1, &x_buf_,
                                 optimized_local_factors_.get(), &sw_pool_,
                                 local_mapping_thread_pool_.get(),
                                 config_.worker_thread_count);
    }
  }

  x_buf_[static_cast<size_t>(win_count_)] = x_curr_;
  pvec_buf_[static_cast<size_t>(win_count_)] = pvec;
  auto* pre = new ImuPreintegration(x_curr_.bg, x_curr_.ba);
  pre->PushImu(imus);
  imu_pres_.push_back(pre);

  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    std::unordered_map<VoxelKey, SlidingOctree*> feat_tem;
    if (sws_.empty()) {
      sws_.resize(1);
    }
    sws_[0] = sw_pool_;
    CutVoxel(feat_map_, pvec, win_count_, feat_tem, config_.win_size, pwld,
             sws_[0]);
    sw_pool_ = sws_[0];
    for (auto& kv : feat_tem) {
      if (kv.second) {
        kv.second->Recut(win_count_ + 1, x_buf_, sw_pool_);
      }
    }
  }

  ++win_count_;
  LocalBaIfReady();
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (history_kfsize_ > 0) {
      for (PreviousMapSession& s : prev_sessions_) {
        if (KeyframeLoading(&feat_map_, s.keyframes, x_curr_.p, 10.0,
                            config_.win_size, &history_kfsize_, jour_)) {
          break;
        }
      }
    }
  }
  SubmitLoopKeyframe(pl);
  SyncStateMirror();

  out->state = state_mirror_;
  out->timestamp = x_curr_.t;
  out->undistorted = std::make_shared<sensor::PointCloudXYZI>(pl);
  out->map_updated = true;
  out->status = ::atlas::odometry::ProcessStatus::kOk;
  return true;
}

}  // namespace vxslam
}  // namespace atlas
