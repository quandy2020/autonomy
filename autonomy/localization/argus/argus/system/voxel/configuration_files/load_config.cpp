/*
 * Copyright 2026 The Openbot Authors
 */

#include "argus/system/voxel/configuration_files/load_config.hpp"

#include <opencv2/core.hpp>

#include "glog/logging.h"

namespace argus {
namespace vxslam {
namespace {

Eigen::Vector3d ReadVec3(const cv::FileNode& n, const Eigen::Vector3d& def) {
    if (n.empty() || !n.isSeq() || n.size() < 3) {
        return def;
    }
    return Eigen::Vector3d(static_cast<double>(n[0]), static_cast<double>(n[1]),
                           static_cast<double>(n[2]));
}

}  // namespace

bool LoadConfig(const std::string& yaml_path,
                                 OdometryConfig* out) {
    if (out == nullptr) {
        return false;
    }
    cv::FileStorage fs(yaml_path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        LOG(ERROR) << "Cannot open " << yaml_path;
        return false;
    }
    OdometryConfig cfg;
    if (!fs["pipeline"].empty()) {
        cfg.pipeline = static_cast<std::string>(fs["pipeline"]);
    }
    const cv::FileNode lid = fs["lidar"];
    if (!lid.empty()) {
        cfg.lidar.blind = static_cast<double>(lid["blind"]);
        cfg.lidar.point_filter_num = static_cast<int>(lid["point_filter_num"]);
        cfg.lidar.synthesize_time =
            static_cast<int>(lid["synthesize_time"]) != 0;
        cfg.lidar.scan_period = static_cast<double>(lid["scan_period"]);
    }
    const cv::FileNode ext = fs["extrinsic"];
    if (!ext.empty()) {
        const cv::FileNode rota = ext["rota"];
        if (rota.isSeq() && rota.size() >= 9) {
            for (int r = 0; r < 3; ++r) {
                for (int c = 0; c < 3; ++c) {
                    cfg.extrinsic_R(r, c) =
                        static_cast<double>(rota[r * 3 + c]);
                }
            }
        }
        cfg.extrinsic_t = ReadVec3(ext["tran"], cfg.extrinsic_t);
    }
    const cv::FileNode od = fs["odometry"];
    if (!od.empty()) {
        cfg.down_size = static_cast<double>(od["down_size"]);
        cfg.voxel_size = static_cast<double>(od["voxel_size"]);
        cfg.min_eigen_value = static_cast<double>(od["min_eigen_value"]);
        cfg.max_layer = static_cast<int>(od["max_layer"]);
        cfg.dept_err = static_cast<double>(od["dept_err"]);
        cfg.beam_err = static_cast<double>(od["beam_err"]);
        cfg.degrade_bound = static_cast<int>(od["degrade_bound"]);
        cfg.point_notime = static_cast<int>(od["point_notime"]) != 0;
        cfg.livox_imu_scale = static_cast<int>(od["livox_imu_scale"]) != 0;
        cfg.cov_gyr = ReadVec3(od["cov_gyr"], cfg.cov_gyr);
        cfg.cov_acc = ReadVec3(od["cov_acc"], cfg.cov_acc);
        const cv::FileNode th = od["plane_eigen_value_thre"];
        if (th.isSeq()) {
            cfg.plane_eigen_value_thre.clear();
            for (int i = 0; i < static_cast<int>(th.size()); ++i) {
                cfg.plane_eigen_value_thre.push_back(
                    static_cast<double>(th[i]));
            }
        }
    }
    const cv::FileNode ba = fs["local_ba"];
    if (!ba.empty()) {
        cfg.win_size = static_cast<int>(ba["win_size"]);
        cfg.imu_coef = static_cast<double>(ba["imu_coef"]);
        cfg.thread_num = static_cast<int>(ba["thread_num"]);
        cfg.enable_local_ba = static_cast<int>(ba["enable"]) != 0;
        cfg.enable_gravity_ba = static_cast<int>(ba["enable_gravity"]) != 0;
    }
    const cv::FileNode loop = fs["loop"];
    if (!loop.empty()) {
        cfg.enable_loop = static_cast<int>(loop["enable"]) != 0;
        cfg.enable_triangle = static_cast<int>(loop["enable_triangle"]) != 0;
        cfg.icp_eigval = static_cast<double>(loop["icp_eigval"]);
        if (!loop["jud_default"].empty()) {
            cfg.jud_default = static_cast<double>(loop["jud_default"]);
        }
        if (!loop["ratio_drift"].empty()) {
            cfg.ratio_drift = static_cast<double>(loop["ratio_drift"]);
        }
        if (!loop["curr_halt"].empty()) {
            cfg.curr_halt = static_cast<int>(loop["curr_halt"]);
        }
        if (!loop["prev_halt"].empty()) {
            cfg.prev_halt = static_cast<int>(loop["prev_halt"]);
        }
    }
    const cv::FileNode gba = fs["gba"];
    if (!gba.empty()) {
        cfg.enable_gba = static_cast<int>(gba["enable"]) != 0;
        if (!gba["local_win"].empty()) {
            cfg.gba_local_win = static_cast<int>(gba["local_win"]);
        }
        if (!gba["merge_size"].empty()) {
            cfg.gba_merge_size = static_cast<int>(gba["merge_size"]);
        }
    }
    const cv::FileNode sess = fs["session"];
    if (!sess.empty()) {
        cfg.save_path = static_cast<std::string>(sess["save_path"]);
        cfg.bagname = static_cast<std::string>(sess["bagname"]);
        cfg.previous_map = static_cast<std::string>(sess["previous_map"]);
    }
    *out = cfg;
    return true;
}

}  // namespace vxslam
}  // namespace argus
