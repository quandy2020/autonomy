/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "autonomy/driver/common/driver_interface.hpp"

#include "autonomy/common/configuration_file_resolver.hpp"
#include "autonomy/common/lua_parameter_dictionary.hpp"

namespace autonomy {
namespace driver {
namespace common {

proto::DriverOptions LoadOptions(::autonomy::common::LuaParameterDictionary* const parameter_dictionary) {
  if (parameter_dictionary == nullptr) {
    proto::DriverOptions options;
    return options;
  }

  proto::DriverOptions options;

  // 加载 auto_start
  if (parameter_dictionary->HasKey("auto_start")) {
    options.set_auto_start(parameter_dictionary->GetBool("auto_start"));
  }

  // 加载 default_forward_targets（字符串数组）
  if (parameter_dictionary->HasKey("default_forward_targets")) {
    auto targets_dict = parameter_dictionary->GetDictionary("default_forward_targets");
    if (targets_dict != nullptr) {
      // 获取数组值作为字符串
      std::vector<std::string> targets = targets_dict->GetArrayValuesAsStrings();
      for (const auto& target : targets) {
        options.add_default_forward_targets(target);
      }
    }
  }

  // 加载 lidars（字典数组）
  if (parameter_dictionary->HasKey("lidars")) {
    auto lidars_dict = parameter_dictionary->GetDictionary("lidars");
    if (lidars_dict != nullptr) {
      std::vector<std::unique_ptr<::autonomy::common::LuaParameterDictionary>> lidar_list =
          lidars_dict->GetArrayValuesAsDictionaries();
      for (const auto& lidar_dict : lidar_list) {
        if (lidar_dict == nullptr) continue;
        auto* lidar = options.add_lidars();

        if (lidar_dict->HasKey("sensor_id")) {
          lidar->set_sensor_id(lidar_dict->GetString("sensor_id"));
        }
        if (lidar_dict->HasKey("type")) {
          lidar->set_type(static_cast<proto::LidarType>(lidar_dict->GetInt("type")));
        }
        if (lidar_dict->HasKey("topic")) {
          lidar->set_topic(lidar_dict->GetString("topic"));
        }
        if (lidar_dict->HasKey("frame_id")) {
          lidar->set_frame_id(lidar_dict->GetString("frame_id"));
        }
        if (lidar_dict->HasKey("enabled")) {
          lidar->set_enabled(lidar_dict->GetBool("enabled"));
        }
        if (lidar_dict->HasKey("min_range")) {
          lidar->set_min_range(lidar_dict->GetDouble("min_range"));
        }
        if (lidar_dict->HasKey("max_range")) {
          lidar->set_max_range(lidar_dict->GetDouble("max_range"));
        }
        if (lidar_dict->HasKey("min_angle")) {
          lidar->set_min_angle(lidar_dict->GetDouble("min_angle"));
        }
        if (lidar_dict->HasKey("max_angle")) {
          lidar->set_max_angle(lidar_dict->GetDouble("max_angle"));
        }
        if (lidar_dict->HasKey("forward_targets")) {
          auto forward_targets_dict = lidar_dict->GetDictionary("forward_targets");
          if (forward_targets_dict != nullptr) {
            std::vector<std::string> targets = forward_targets_dict->GetArrayValuesAsStrings();
            for (const auto& target : targets) {
              lidar->add_forward_targets(target);
            }
          }
        }
      }
    }
  }

  // 加载 imus（字典数组）
  if (parameter_dictionary->HasKey("imus")) {
    auto imus_dict = parameter_dictionary->GetDictionary("imus");
    if (imus_dict != nullptr) {
      std::vector<std::unique_ptr<::autonomy::common::LuaParameterDictionary>> imu_list =
          imus_dict->GetArrayValuesAsDictionaries();
      for (const auto& imu_dict : imu_list) {
        if (imu_dict == nullptr) continue;
        auto* imu = options.add_imus();

        if (imu_dict->HasKey("sensor_id")) {
          imu->set_sensor_id(imu_dict->GetString("sensor_id"));
        }
        if (imu_dict->HasKey("topic")) {
          imu->set_topic(imu_dict->GetString("topic"));
        }
        if (imu_dict->HasKey("frame_id")) {
          imu->set_frame_id(imu_dict->GetString("frame_id"));
        }
        if (imu_dict->HasKey("enabled")) {
          imu->set_enabled(imu_dict->GetBool("enabled"));
        }
        if (imu_dict->HasKey("angular_velocity_covariance_scale")) {
          imu->set_angular_velocity_covariance_scale(imu_dict->GetDouble("angular_velocity_covariance_scale"));
        }
        if (imu_dict->HasKey("linear_acceleration_covariance_scale")) {
          imu->set_linear_acceleration_covariance_scale(imu_dict->GetDouble("linear_acceleration_covariance_scale"));
        }
        if (imu_dict->HasKey("orientation_covariance_scale")) {
          imu->set_orientation_covariance_scale(imu_dict->GetDouble("orientation_covariance_scale"));
        }
        if (imu_dict->HasKey("forward_targets")) {
          auto forward_targets_dict = imu_dict->GetDictionary("forward_targets");
          if (forward_targets_dict != nullptr) {
            std::vector<std::string> targets = forward_targets_dict->GetArrayValuesAsStrings();
            for (const auto& target : targets) {
              imu->add_forward_targets(target);
            }
          }
        }
        if (imu_dict->HasKey("sampling_rate")) {
          imu->set_sampling_rate(imu_dict->GetDouble("sampling_rate"));
        }
      }
    }
  }

  // 加载 cameras（字典数组）
  if (parameter_dictionary->HasKey("cameras")) {
    auto cameras_dict = parameter_dictionary->GetDictionary("cameras");
    if (cameras_dict != nullptr) {
      std::vector<std::unique_ptr<::autonomy::common::LuaParameterDictionary>> camera_list =
          cameras_dict->GetArrayValuesAsDictionaries();
      for (const auto& camera_dict : camera_list) {
        if (camera_dict == nullptr) continue;
        auto* camera = options.add_cameras();

        if (camera_dict->HasKey("sensor_id")) {
          camera->set_sensor_id(camera_dict->GetString("sensor_id"));
        }
        if (camera_dict->HasKey("type")) {
          camera->set_type(static_cast<proto::CameraType>(camera_dict->GetInt("type")));
        }
        if (camera_dict->HasKey("image_topic")) {
          camera->set_image_topic(camera_dict->GetString("image_topic"));
        }
        if (camera_dict->HasKey("camera_info_topic")) {
          camera->set_camera_info_topic(camera_dict->GetString("camera_info_topic"));
        }
        if (camera_dict->HasKey("depth_topic")) {
          camera->set_depth_topic(camera_dict->GetString("depth_topic"));
        }
        if (camera_dict->HasKey("frame_id")) {
          camera->set_frame_id(camera_dict->GetString("frame_id"));
        }
        if (camera_dict->HasKey("enabled")) {
          camera->set_enabled(camera_dict->GetBool("enabled"));
        }
        if (camera_dict->HasKey("width")) {
          camera->set_width(static_cast<uint32_t>(camera_dict->GetInt("width")));
        }
        if (camera_dict->HasKey("height")) {
          camera->set_height(static_cast<uint32_t>(camera_dict->GetInt("height")));
        }
        if (camera_dict->HasKey("encoding")) {
          camera->set_encoding(camera_dict->GetString("encoding"));
        }
        if (camera_dict->HasKey("forward_targets")) {
          auto forward_targets_dict = camera_dict->GetDictionary("forward_targets");
          if (forward_targets_dict != nullptr) {
            std::vector<std::string> targets = forward_targets_dict->GetArrayValuesAsStrings();
            for (const auto& target : targets) {
              camera->add_forward_targets(target);
            }
          }
        }
        if (camera_dict->HasKey("sampling_rate")) {
          camera->set_sampling_rate(camera_dict->GetDouble("sampling_rate"));
        }
      }
    }
  }

  // 加载 ranges（字典数组）
  if (parameter_dictionary->HasKey("ranges")) {
    auto ranges_dict = parameter_dictionary->GetDictionary("ranges");
    if (ranges_dict != nullptr) {
      std::vector<std::unique_ptr<::autonomy::common::LuaParameterDictionary>> range_list =
          ranges_dict->GetArrayValuesAsDictionaries();
      for (const auto& range_dict : range_list) {
        if (range_dict == nullptr) continue;
        auto* range = options.add_ranges();

        if (range_dict->HasKey("sensor_id")) {
          range->set_sensor_id(range_dict->GetString("sensor_id"));
        }
        if (range_dict->HasKey("topic")) {
          range->set_topic(range_dict->GetString("topic"));
        }
        if (range_dict->HasKey("frame_id")) {
          range->set_frame_id(range_dict->GetString("frame_id"));
        }
        if (range_dict->HasKey("enabled")) {
          range->set_enabled(range_dict->GetBool("enabled"));
        }
        if (range_dict->HasKey("radiation_type")) {
          range->set_radiation_type(static_cast<proto::RangeRadiationType>(range_dict->GetInt("radiation_type")));
        }
        if (range_dict->HasKey("field_of_view")) {
          range->set_field_of_view(range_dict->GetDouble("field_of_view"));
        }
        if (range_dict->HasKey("min_range")) {
          range->set_min_range(range_dict->GetDouble("min_range"));
        }
        if (range_dict->HasKey("max_range")) {
          range->set_max_range(range_dict->GetDouble("max_range"));
        }
        if (range_dict->HasKey("forward_targets")) {
          auto forward_targets_dict = range_dict->GetDictionary("forward_targets");
          if (forward_targets_dict != nullptr) {
            std::vector<std::string> targets = forward_targets_dict->GetArrayValuesAsStrings();
            for (const auto& target : targets) {
              range->add_forward_targets(target);
            }
          }
        }
        if (range_dict->HasKey("sampling_rate")) {
          range->set_sampling_rate(range_dict->GetDouble("sampling_rate"));
        }
      }
    }
  }

  // 加载 gps_sensors（字典数组）
  if (parameter_dictionary->HasKey("gps_sensors")) {
    auto gps_dict = parameter_dictionary->GetDictionary("gps_sensors");
    if (gps_dict != nullptr) {
      std::vector<std::unique_ptr<::autonomy::common::LuaParameterDictionary>> gps_list =
          gps_dict->GetArrayValuesAsDictionaries();
      for (const auto& gps_sensor_dict : gps_list) {
        if (gps_sensor_dict == nullptr) continue;
        auto* gps = options.add_gps_sensors();

        if (gps_sensor_dict->HasKey("sensor_id")) {
          gps->set_sensor_id(gps_sensor_dict->GetString("sensor_id"));
        }
        if (gps_sensor_dict->HasKey("topic")) {
          gps->set_topic(gps_sensor_dict->GetString("topic"));
        }
        if (gps_sensor_dict->HasKey("frame_id")) {
          gps->set_frame_id(gps_sensor_dict->GetString("frame_id"));
        }
        if (gps_sensor_dict->HasKey("enabled")) {
          gps->set_enabled(gps_sensor_dict->GetBool("enabled"));
        }
        if (gps_sensor_dict->HasKey("min_position_accuracy")) {
          gps->set_min_position_accuracy(gps_sensor_dict->GetDouble("min_position_accuracy"));
        }
        if (gps_sensor_dict->HasKey("min_altitude_accuracy")) {
          gps->set_min_altitude_accuracy(gps_sensor_dict->GetDouble("min_altitude_accuracy"));
        }
        if (gps_sensor_dict->HasKey("forward_targets")) {
          auto forward_targets_dict = gps_sensor_dict->GetDictionary("forward_targets");
          if (forward_targets_dict != nullptr) {
            std::vector<std::string> targets = forward_targets_dict->GetArrayValuesAsStrings();
            for (const auto& target : targets) {
              gps->add_forward_targets(target);
            }
          }
        }
        if (gps_sensor_dict->HasKey("sampling_rate")) {
          gps->set_sampling_rate(gps_sensor_dict->GetDouble("sampling_rate"));
        }
        if (gps_sensor_dict->HasKey("use_for_localization")) {
          gps->set_use_for_localization(gps_sensor_dict->GetBool("use_for_localization"));
        }
        if (gps_sensor_dict->HasKey("service_type")) {
          gps->set_service_type(static_cast<uint32_t>(gps_sensor_dict->GetInt("service_type")));
        }
      }
    }
  }

  return options;
}

proto::DriverOptions CreateOptions(const std::string& configuration_directory,
                                   const std::string& configuration_basename) {
  auto file_resolver = std::make_unique<::autonomy::common::ConfigurationFileResolver>(
      std::vector<std::string>{configuration_directory});
  std::string code = file_resolver->GetFileContentOrDie(configuration_basename);
  ::autonomy::common::LuaParameterDictionary parameter_dictionary(code, std::move(file_resolver));
  return LoadOptions(parameter_dictionary.GetDictionary("driver").get());
}

}  // namespace common
}  // namespace driver
}  // namespace autonomy
