#ifndef FASTER_LIO_POINTCLOUD_PROCESSING_H
#define FASTER_LIO_POINTCLOUD_PROCESSING_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "common/point_def.hpp"

namespace lightning {

enum class LidarType { VELO32 = 2, OUST64 = 3, ROBOSENSE = 4, GENERIC = 5 };

inline bool LidarTypeFromInt(int lidar_type, LidarType* out) {
    switch (lidar_type) {
        case static_cast<int>(LidarType::VELO32):
            *out = LidarType::VELO32;
            return true;
        case static_cast<int>(LidarType::OUST64):
            *out = LidarType::OUST64;
            return true;
        case static_cast<int>(LidarType::ROBOSENSE):
            *out = LidarType::ROBOSENSE;
            return true;
        case static_cast<int>(LidarType::GENERIC):
            *out = LidarType::GENERIC;
            return true;
        default:
            return false;
    }
}

/**
 * Unify / filter a scan into lightning::PointXYZIT.
 * ROS-free: callers convert automsgs PointCloud2 (or other sources) to CloudPtr.
 */
class PointCloudPreprocess {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PointCloudPreprocess() = default;
    ~PointCloudPreprocess() = default;

    void Process(const CloudPtr& cloud_in, PointCloudType::Ptr& pcl_out);

    void Set(LidarType lid_type, double bld, int pfilt_num);

    double& Blind() { return blind_; }
    int& NumScans() { return num_scans_; }
    int& PointFilterNum() { return point_filter_num_; }
    float& TimeScale() { return time_scale_; }
    LidarType GetLidarType() const { return lidar_type_; }
    void SetLidarType(LidarType lt) { lidar_type_ = lt; }

    void SetHeightROI(float height_max, float height_min) {
        height_max_ = height_max;
        height_min_ = height_min;
    }

   private:
    PointCloudType cloud_out_;

    LidarType lidar_type_ = LidarType::GENERIC;
    int point_filter_num_ = 1;
    int num_scans_ = 6;
    double blind_ = 0.01;
    float time_scale_ = 1e-3;

    float height_max_ = 1.0;
    float height_min_ = -1.0;
};
}  // namespace lightning

#endif
