/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ARGUS_VXSLAM_INIT_HPP_
#define ARGUS_VXSLAM_INIT_HPP_

#include <deque>
#include <unordered_map>
#include <vector>

#include "frontend/lidar/imu/imu_preintegration.hpp"
#include "frontend/lidar/imu/imu_state.hpp"
#include "frontend/lidar/local_mapping/lidar_factor.hpp"
#include "frontend/lidar/odometry/point_var.hpp"
#include "map/voxel/sliding_octree.hpp"
#include "map/voxel/voxel_key_hash.hpp"
#include "sensor/imu_types.hpp"
#include "sensor/point_types.hpp"

namespace argus {
namespace vxslam {

/** @brief Seed gravity from mean accel (simple IMU align). */
bool AlignGravity(const std::deque<sensor::ImuData>& imus, ImuState* x,
                  double scale_gravity = 1.0);

/** @brief True when enough scans buffered for MotionInit. */
bool MotionInitReady(int buffered_scans, int min_scans = 5);

/** @brief Rotate window states so gravity aligns with ±z (align_gravity). */
void AlignGravityStates(std::vector<ImuState>* xs);

/**
 * @brief Init-time motion undistortion into PointVar list (motion_blur).
 * @param point_notime If true, skip time-based undistortion.
 */
void MotionBlur(const sensor::PointCloudXYZI& pl, PointVarList* pvec,
                ImuState xc, const ImuState& xl,
                const std::deque<sensor::ImuData>& imus, double pcl_beg_time,
                const ImuState& extrin, bool point_notime);

struct MotionInitInput {
  std::vector<sensor::PointCloudXYZI::Ptr> clouds;
  std::vector<std::deque<sensor::ImuData>> imus;
  std::vector<double> beg_times;
  ImuState extrin;
  double dept_err = 0.05;
  double beam_err = 0.2;
  bool point_notime = false;
};

/**
 * @brief Full Voxel-SLAM motion_init (gravity LI-BA + plane coverage).
 * @return 1 on success, 0 on degrade / failed gravity.
 */
int MotionInit(MotionInitInput* input, Eigen::MatrixXd* hess,
               LidarFactor* voxhess, std::vector<ImuState>* x_buf,
               std::unordered_map<VoxelKey, SlidingOctree*>* surf_map,
               std::unordered_map<VoxelKey, SlidingOctree*>* surf_map_slide,
               std::vector<PointVarListPtr>* pvec_buf, int win_size,
               std::vector<std::vector<FrameWindow*>>* sws, ImuState* x_curr,
               std::deque<ImuPreintegration*>* imu_pre_buf);

}  // namespace vxslam
}  // namespace argus

#endif
