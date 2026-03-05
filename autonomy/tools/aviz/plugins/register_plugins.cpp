/******************************************************************************
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "autonomy/tools/aviz/plugins/register_plugins.hpp"

#include "autonomy/tools/aviz/common/display.hpp"
#include "autonomy/tools/aviz/plugins/plugin_factory.hpp"
#include "autonomy/tools/aviz/robot_model_display.hpp"

// Display plugin includes (from displays/ directory)
#include "autonomy/tools/aviz/plugins/displays/accel/accel_display.hpp"
#include "autonomy/tools/aviz/plugins/displays/axes/axes_display.hpp"
#include "autonomy/tools/aviz/plugins/displays/camera/camera_display.hpp"
#include "autonomy/tools/aviz/plugins/displays/depth_cloud/depth_cloud_display.hpp"
#include "autonomy/tools/aviz/plugins/displays/effort/effort_display.hpp"
#include "autonomy/tools/aviz/plugins/displays/fluid_pressure/fluid_pressure_display.hpp"
#include "autonomy/tools/aviz/plugins/displays/grid/grid_display.hpp"
#include "autonomy/tools/aviz/plugins/displays/grid_cells/grid_cells_display.hpp"
#include "autonomy/tools/aviz/plugins/displays/illuminance/illuminance_display.hpp"
#include "autonomy/tools/aviz/plugins/displays/image/image_display.hpp"
#include "autonomy/tools/aviz/plugins/displays/interactive_markers/interactive_markers_display.hpp"
#include "autonomy/tools/aviz/plugins/displays/laser_scan/laser_scan_display.hpp"
#include "autonomy/tools/aviz/plugins/displays/map/map_display.hpp"
#include "autonomy/tools/aviz/plugins/displays/marker/marker_display.hpp"
#include "autonomy/tools/aviz/plugins/displays/marker_array/marker_array_display.hpp"
#include "autonomy/tools/aviz/plugins/displays/odometry/odometry_display.hpp"
#include "autonomy/tools/aviz/plugins/displays/path/path_display.hpp"
#include "autonomy/tools/aviz/plugins/displays/point/point_display.hpp"
#include "autonomy/tools/aviz/plugins/displays/polygon/polygon_display.hpp"
#include "autonomy/tools/aviz/plugins/displays/pose/pose_display.hpp"
#include "autonomy/tools/aviz/plugins/displays/pose_array/pose_array_display.hpp"
#include "autonomy/tools/aviz/plugins/displays/pose_covariance/pose_covariance_display.hpp"
#include "autonomy/tools/aviz/plugins/displays/range/range_display.hpp"
#include "autonomy/tools/aviz/plugins/displays/relative_humidity/relative_humidity_display.hpp"
#include "autonomy/tools/aviz/plugins/displays/robot_model/robot_model_display.hpp"
#include "autonomy/tools/aviz/plugins/displays/screw/screw_display.hpp"
#include "autonomy/tools/aviz/plugins/displays/temperature/temperature_display.hpp"
#include "autonomy/tools/aviz/plugins/displays/tf/tf_display.hpp"
#include "autonomy/tools/aviz/plugins/displays/twist/twist_display.hpp"
#include "autonomy/tools/aviz/plugins/displays/wrench/wrench_display.hpp"

void RegisterDefaultPlugins() {
  auto& factory = PluginFactory::Instance();

  // Register RobotModelDisplay (top-level class, not in namespace)
  {
    PluginDescription desc;
    desc.class_id = "aviz/RobotModel";
    desc.class_name = "RobotModelDisplay";
    desc.description = "Displays a visual representation of a robot from URDF files.";
    desc.base_class = "Display";
    desc.message_types.clear();

    factory.registerPlugin(desc, [](const std::string& name) -> std::unique_ptr<aviz::common::Display> {
      return std::make_unique<RobotModelDisplay>(name);
    });
  }

  // Register all display plugins from aviz::plugins::displays namespace
  using namespace aviz::plugins::displays;

  // Register AccelDisplay
  {
    PluginDescription desc;
    desc.class_id = "aviz/Accel";
    desc.class_name = "AccelDisplay";
    desc.description = "Acceleration display";
    desc.base_class = "Display";
    desc.message_types.clear();
    factory.registerPlugin(desc, [](const std::string& name) -> std::unique_ptr<aviz::common::Display> {
      return std::make_unique<AccelDisplay>(QString::fromStdString(name));
    });
  }

  // Register AxesDisplay
  {
    PluginDescription desc;
    desc.class_id = "aviz/Axes";
    desc.class_name = "AxesDisplay";
    desc.description = "Axes display";
    desc.base_class = "Display";
    desc.message_types.clear();
    factory.registerPlugin(desc, [](const std::string& name) -> std::unique_ptr<aviz::common::Display> {
      return std::make_unique<AxesDisplay>();
    });
  }

  // Register CameraDisplay
  {
    PluginDescription desc;
    desc.class_id = "aviz/Camera";
    desc.class_name = "CameraDisplay";
    desc.description = "Camera display";
    desc.base_class = "Display";
    desc.message_types.clear();
    factory.registerPlugin(desc, [](const std::string& name) -> std::unique_ptr<aviz::common::Display> {
      return std::make_unique<CameraDisplay>(QString::fromStdString(name));
    });
  }

  // Register DepthCloudDisplay
  {
    PluginDescription desc;
    desc.class_id = "aviz/DepthCloud";
    desc.class_name = "DepthCloudDisplay";
    desc.description = "Depth cloud display";
    desc.base_class = "Display";
    desc.message_types.clear();
    factory.registerPlugin(desc, [](const std::string& name) -> std::unique_ptr<aviz::common::Display> {
      return std::make_unique<DepthCloudDisplay>(QString::fromStdString(name));
    });
  }

  // Register EffortDisplay
  {
    PluginDescription desc;
    desc.class_id = "aviz/Effort";
    desc.class_name = "EffortDisplay";
    desc.description = "Effort display";
    desc.base_class = "Display";
    desc.message_types.clear();
    factory.registerPlugin(desc, [](const std::string& name) -> std::unique_ptr<aviz::common::Display> {
      return std::make_unique<EffortDisplay>(QString::fromStdString(name));
    });
  }

  // Register FluidPressureDisplay
  {
    PluginDescription desc;
    desc.class_id = "aviz/FluidPressure";
    desc.class_name = "FluidPressureDisplay";
    desc.description = "Fluid pressure display";
    desc.base_class = "Display";
    desc.message_types.clear();
    factory.registerPlugin(desc, [](const std::string& name) -> std::unique_ptr<aviz::common::Display> {
      return std::make_unique<FluidPressureDisplay>(QString::fromStdString(name));
    });
  }

  // Register GridDisplay
  {
    PluginDescription desc;
    desc.class_id = "aviz/Grid";
    desc.class_name = "GridDisplay";
    desc.description = "Grid display";
    desc.base_class = "Display";
    desc.message_types.clear();
    factory.registerPlugin(desc, [](const std::string& name) -> std::unique_ptr<aviz::common::Display> {
      return std::make_unique<GridDisplay>();
    });
  }

  // Register GridCellsDisplay
  {
    PluginDescription desc;
    desc.class_id = "aviz/GridCells";
    desc.class_name = "GridCellsDisplay";
    desc.description = "Grid cells display";
    desc.base_class = "Display";
    desc.message_types.clear();
    factory.registerPlugin(desc, [](const std::string& name) -> std::unique_ptr<aviz::common::Display> {
      return std::make_unique<GridCellsDisplay>(QString::fromStdString(name));
    });
  }

  // Register IlluminanceDisplay
  {
    PluginDescription desc;
    desc.class_id = "aviz/Illuminance";
    desc.class_name = "IlluminanceDisplay";
    desc.description = "Illuminance display";
    desc.base_class = "Display";
    desc.message_types.clear();
    factory.registerPlugin(desc, [](const std::string& name) -> std::unique_ptr<aviz::common::Display> {
      return std::make_unique<IlluminanceDisplay>(QString::fromStdString(name));
    });
  }

  // Register ImageDisplay
  {
    PluginDescription desc;
    desc.class_id = "aviz/Image";
    desc.class_name = "ImageDisplay";
    desc.description = "Image display";
    desc.base_class = "Display";
    desc.message_types.clear();
    factory.registerPlugin(desc, [](const std::string& name) -> std::unique_ptr<aviz::common::Display> {
      return std::make_unique<ImageDisplay>(QString::fromStdString(name));
    });
  }

  // Register InteractiveMarkerDisplay
  {
    PluginDescription desc;
    desc.class_id = "aviz/InteractiveMarker";
    desc.class_name = "InteractiveMarkerDisplay";
    desc.description = "Interactive marker display";
    desc.base_class = "Display";
    desc.message_types.clear();
    factory.registerPlugin(desc, [](const std::string& name) -> std::unique_ptr<aviz::common::Display> {
      return std::make_unique<aviz::plugins::displays::InteractiveMarkersDisplay>(QString::fromStdString(name));
    });
  }

  // Register LaserScanDisplay
  {
    PluginDescription desc;
    desc.class_id = "aviz/LaserScan";
    desc.class_name = "LaserScanDisplay";
    desc.description = "Laser scan display";
    desc.base_class = "Display";
    desc.message_types.clear();
    factory.registerPlugin(desc, [](const std::string& name) -> std::unique_ptr<aviz::common::Display> {
      return std::make_unique<LaserScanDisplay>(QString::fromStdString(name));
    });
  }

  // Register MapDisplay
  {
    PluginDescription desc;
    desc.class_id = "aviz/Map";
    desc.class_name = "MapDisplay";
    desc.description = "Map display";
    desc.base_class = "Display";
    desc.message_types.clear();
    factory.registerPlugin(desc, [](const std::string& name) -> std::unique_ptr<aviz::common::Display> {
      return std::make_unique<MapDisplay>(QString::fromStdString(name));
    });
  }

  // Register MarkerDisplay
  {
    PluginDescription desc;
    desc.class_id = "aviz/Marker";
    desc.class_name = "MarkerDisplay";
    desc.description = "Marker display";
    desc.base_class = "Display";
    desc.message_types.clear();
    factory.registerPlugin(desc, [](const std::string& name) -> std::unique_ptr<aviz::common::Display> {
      return std::make_unique<MarkerDisplay>(QString::fromStdString(name));
    });
  }

  // Register MarkerArrayDisplay
  {
    PluginDescription desc;
    desc.class_id = "aviz/MarkerArray";
    desc.class_name = "MarkerArrayDisplay";
    desc.description = "Marker array display";
    desc.base_class = "Display";
    desc.message_types.clear();
    factory.registerPlugin(desc, [](const std::string& name) -> std::unique_ptr<aviz::common::Display> {
      return std::make_unique<MarkerArrayDisplay>(QString::fromStdString(name));
    });
  }

  // Register OdometryDisplay
  {
    PluginDescription desc;
    desc.class_id = "aviz/Odometry";
    desc.class_name = "OdometryDisplay";
    desc.description = "Odometry display";
    desc.base_class = "Display";
    desc.message_types.clear();
    factory.registerPlugin(desc, [](const std::string& name) -> std::unique_ptr<aviz::common::Display> {
      return std::make_unique<OdometryDisplay>();
    });
  }

  // Register PathDisplay
  {
    PluginDescription desc;
    desc.class_id = "aviz/Path";
    desc.class_name = "PathDisplay";
    desc.description = "Path display";
    desc.base_class = "Display";
    desc.message_types.clear();
    factory.registerPlugin(desc, [](const std::string& name) -> std::unique_ptr<aviz::common::Display> {
      return std::make_unique<PathDisplay>();
    });
  }

  // Register PointDisplay
  {
    PluginDescription desc;
    desc.class_id = "aviz/Point";
    desc.class_name = "PointDisplay";
    desc.description = "Point display";
    desc.base_class = "Display";
    desc.message_types.clear();
    factory.registerPlugin(desc, [](const std::string& name) -> std::unique_ptr<aviz::common::Display> {
      return std::make_unique<PointDisplay>(QString::fromStdString(name));
    });
  }

  // Register PolygonDisplay
  {
    PluginDescription desc;
    desc.class_id = "aviz/Polygon";
    desc.class_name = "PolygonDisplay";
    desc.description = "Polygon display";
    desc.base_class = "Display";
    desc.message_types.clear();
    factory.registerPlugin(desc, [](const std::string& name) -> std::unique_ptr<aviz::common::Display> {
      return std::make_unique<PolygonDisplay>(QString::fromStdString(name));
    });
  }

  // Register PoseDisplay
  {
    PluginDescription desc;
    desc.class_id = "aviz/Pose";
    desc.class_name = "PoseDisplay";
    desc.description = "Pose display";
    desc.base_class = "Display";
    desc.message_types.clear();
    factory.registerPlugin(desc, [](const std::string& name) -> std::unique_ptr<aviz::common::Display> {
      return std::make_unique<PoseDisplay>();
    });
  }

  // Register PoseArrayDisplay
  {
    PluginDescription desc;
    desc.class_id = "aviz/PoseArray";
    desc.class_name = "PoseArrayDisplay";
    desc.description = "Pose array display";
    desc.base_class = "Display";
    desc.message_types.clear();
    factory.registerPlugin(desc, [](const std::string& name) -> std::unique_ptr<aviz::common::Display> {
      return std::make_unique<PoseArrayDisplay>(QString::fromStdString(name));
    });
  }

  // Register PoseWithCovarianceDisplay
  {
    PluginDescription desc;
    desc.class_id = "aviz/PoseWithCovariance";
    desc.class_name = "PoseWithCovarianceDisplay";
    desc.description = "Pose with covariance display";
    desc.base_class = "Display";
    desc.message_types.clear();
    factory.registerPlugin(desc, [](const std::string& name) -> std::unique_ptr<aviz::common::Display> {
      return std::make_unique<aviz::plugins::displays::PoseCovarianceDisplay>(QString::fromStdString(name));
    });
  }

  // Register RangeDisplay
  {
    PluginDescription desc;
    desc.class_id = "aviz/Range";
    desc.class_name = "RangeDisplay";
    desc.description = "Range display";
    desc.base_class = "Display";
    desc.message_types.clear();
    factory.registerPlugin(desc, [](const std::string& name) -> std::unique_ptr<aviz::common::Display> {
      return std::make_unique<RangeDisplay>(QString::fromStdString(name));
    });
  }

  // Register RelativeHumidityDisplay
  {
    PluginDescription desc;
    desc.class_id = "aviz/RelativeHumidity";
    desc.class_name = "RelativeHumidityDisplay";
    desc.description = "Relative humidity display";
    desc.base_class = "Display";
    desc.message_types.clear();
    factory.registerPlugin(desc, [](const std::string& name) -> std::unique_ptr<aviz::common::Display> {
      return std::make_unique<RelativeHumidityDisplay>(QString::fromStdString(name));
    });
  }

  // Note: RobotModelDisplay from displays/robot_model/robot_model_display.hpp
  // is in aviz::plugins::displays namespace, but we already registered the top-level one above.

  // Register ScrewDisplay
  {
    PluginDescription desc;
    desc.class_id = "aviz/Screw";
    desc.class_name = "ScrewDisplay";
    desc.description = "Screw display";
    desc.base_class = "Display";
    desc.message_types.clear();
    factory.registerPlugin(desc, [](const std::string& name) -> std::unique_ptr<aviz::common::Display> {
      return std::make_unique<ScrewDisplay>(QString::fromStdString(name));
    });
  }

  // Register TemperatureDisplay
  {
    PluginDescription desc;
    desc.class_id = "aviz/Temperature";
    desc.class_name = "TemperatureDisplay";
    desc.description = "Temperature display";
    desc.base_class = "Display";
    desc.message_types.clear();
    factory.registerPlugin(desc, [](const std::string& name) -> std::unique_ptr<aviz::common::Display> {
      return std::make_unique<TemperatureDisplay>(QString::fromStdString(name));
    });
  }

  // Register TFDisplay
  {
    PluginDescription desc;
    desc.class_id = "aviz/TF";
    desc.class_name = "TFDisplay";
    desc.description = "TF display";
    desc.base_class = "Display";
    desc.message_types.clear();
    factory.registerPlugin(desc, [](const std::string& name) -> std::unique_ptr<aviz::common::Display> {
      return std::make_unique<aviz::plugins::displays::TfDisplay>(QString::fromStdString(name));
    });
  }

  // Register TwistDisplay
  {
    PluginDescription desc;
    desc.class_id = "aviz/Twist";
    desc.class_name = "TwistDisplay";
    desc.description = "Twist display";
    desc.base_class = "Display";
    desc.message_types.clear();
    factory.registerPlugin(desc, [](const std::string& name) -> std::unique_ptr<aviz::common::Display> {
      return std::make_unique<TwistDisplay>(QString::fromStdString(name));
    });
  }

  // Register WrenchDisplay
  {
    PluginDescription desc;
    desc.class_id = "aviz/Wrench";
    desc.class_name = "WrenchDisplay";
    desc.description = "Wrench display";
    desc.base_class = "Display";
    desc.message_types.clear();
    factory.registerPlugin(desc, [](const std::string& name) -> std::unique_ptr<aviz::common::Display> {
      return std::make_unique<WrenchDisplay>(QString::fromStdString(name));
    });
  }
}
