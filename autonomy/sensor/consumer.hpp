/*
 * Copyright 2026 autonomy contributors
 */

#pragma once

#include <functional>

#include "autonomy/control/controller_server.hpp"
#include "autonomy/sensor/laser_data.hpp"
#include "autonomy/sensor/odometry_data.hpp"
#include "autonomy/sensor/point_cloud.hpp"
#include "autonomy/sensor/range_data.hpp"

namespace autonomy {
namespace sensor {

/** @brief Sensor sinks; map/costmap wiring lives outside this module (e.g. Task). */
class SensorConsumer
{
public:
  control::ControllerServer * controller{nullptr};

  std::function<void(const LaserScanData &)> on_laser_scan;
  std::function<void(const PointCloud2Data &)> on_point_cloud;
  std::function<void(const RangeData &)> on_range;

  void OnOdometry(const OdometryData & odom) const
  {
    if (controller) {
      controller->UpdateOdometry(odom);
    }
  }

  void OnLaserScan(const LaserScanData & scan) const
  {
    if (on_laser_scan) {
      on_laser_scan(scan);
    }
  }

  void OnPointCloud2(const PointCloud2Data & cloud) const
  {
    if (on_point_cloud) {
      on_point_cloud(cloud);
    }
  }

  void OnRange(const RangeData & range) const
  {
    if (on_range) {
      on_range(range);
    }
  }
};

}  // namespace sensor
}  // namespace autonomy
