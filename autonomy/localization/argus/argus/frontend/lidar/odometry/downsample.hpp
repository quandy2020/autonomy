/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ARGUS_VXSLAM_DOWNSAMPLE_HPP_
#define ARGUS_VXSLAM_DOWNSAMPLE_HPP_

#include "frontend/lidar/odometry/point_var.hpp"
#include "sensor/point_types.hpp"

namespace argus {
namespace vxslam {

/** @brief Voxel-average downsample of an XYZINormal cloud. */
void DownsampleVoxel(sensor::PointCloudXYZI* cloud, double voxel_size);

/**
 * @brief Keep the point nearest to each voxel centroid
 *        (Voxel-SLAM down_sampling_close).
 */
void DownsampleClose(sensor::PointCloudXYZI* cloud, double voxel_size);

/** @brief Voxel-average downsample of PointVar list into a cloud. */
void DownsamplePointVars(const PointVarList& points, double voxel_size,
                         sensor::PointCloudXYZI* out);

}  // namespace vxslam
}  // namespace argus

#endif
