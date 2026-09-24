//
// Created by xiang on 2021/11/17.
//
#pragma once

#include "autonomy/localization/atlas/port/common/eigen_types.hpp"
#include "autonomy/localization/atlas/port/common/nav_state.hpp"

namespace atlas_lio::loc {

enum class LocalizationStatus {
    IDLE,
    INITIALIZING,
    GOOD,
    FOLLOWING_DR,
    FAIL,
};

struct LocalizationResult {
    double timestamp_ = 0;
    SE3 pose_;
    bool valid_ = false;
    LocalizationStatus status_ = LocalizationStatus::IDLE;

    bool lidar_loc_valid_ = false;
    bool lidar_loc_inlier_ = false;
    double confidence_ = 1.0;
    double lidar_loc_error_vert_ = 0;
    double lidar_loc_error_hori_ = 0;
    double lidar_loc_delta_t_ = 0;
    double lidar_loc_odom_delta_ = 0;
    bool lidar_loc_smooth_flag_ = false;

    bool lidar_loc_odom_error_normal_ = true;
    bool lidar_loc_odom_reliable_ = true;

    double lidar_odom_error_vert_ = 0;
    double lidar_odom_error_hori_ = 0;

    bool rel_pose_set_ = false;
    SE3 rel_pose_;
    Vec3d vel_b_ = Vec3d::Zero();
    double lidar_odom_delta_t_ = 0;
    double dr_delta_t_ = 0;
    double is_parking_ = false;

    NavState ToNavState() const;
};

}  // namespace atlas_lio::loc
