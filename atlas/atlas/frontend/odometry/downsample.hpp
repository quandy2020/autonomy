/*
 * Copyright 2026 The Openbot Authors
 */
#ifndef ATLAS_FRONTEND_ODOMETRY_DOWNSAMPLE_HPP_
#define ATLAS_FRONTEND_ODOMETRY_DOWNSAMPLE_HPP_

#include "map/point_observation.hpp"
#include "sensor/point_types.hpp"

namespace atlas {
namespace frontend {
namespace odometry {

using ::atlas::map::PointObservation;
using ::atlas::map::PointObservationList;
using ::atlas::map::SharedPointObservationList;

/** @brief Voxel-average downsample of an XYZINormal cloud. */
void DownsampleVoxel(sensor::PointCloudXYZI* cloud, double voxel_size);

/**
 * @brief Keep the point nearest to each voxel centroid
 *        (Voxel-SLAM down_sampling_close).
 */
void DownsampleClose(sensor::PointCloudXYZI* cloud, double voxel_size);

/** @brief Prepares raw scans retained for motion initialization. */
void DownsampleInitializationScan(sensor::PointCloudXYZI* cloud,
                                  double voxel_size);

/** @brief Voxel-average downsample of PointObservation list into a cloud. */
void DownsamplePointObservations(const PointObservationList& points,
                                 double voxel_size,
                                 sensor::PointCloudXYZI* out);

}  // namespace odometry
}  // namespace frontend
}  // namespace atlas

#endif
