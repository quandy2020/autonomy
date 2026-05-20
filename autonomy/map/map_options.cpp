/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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

#include "autonomy/map/map_options.hpp"

#include "autonomy/common/configuration_file_resolver.hpp"
#include "autonomy/common/lua_parameter_dictionary.hpp"

namespace autonomy {
namespace map {

proto::MapOptions LoadOptions(
    ::autonomy::common::LuaParameterDictionary* const parameter_dictionary) {
    proto::MapOptions options;
    options.set_map_name(parameter_dictionary->GetString("map_name"));
    options.set_map_file(parameter_dictionary->GetString("map_file"));
    options.set_map_topic(parameter_dictionary->GetString("map_topic"));
    options.set_publish_frequency(
        parameter_dictionary->GetDouble("publish_frequency"));
    options.set_frame_id(parameter_dictionary->GetString("frame_id"));
    return options;
}

proto::Costmap2DOptions CreateCostmap2DOptions(
    ::autonomy::common::LuaParameterDictionary* const parameter_dictionary) {
    proto::Costmap2DOptions options;
    if (parameter_dictionary->HasKey("enabled")) {
        options.set_enabled(parameter_dictionary->GetBool("enabled"));
    }
    options.set_frame_id(parameter_dictionary->GetString("frame_id"));
    options.set_name(parameter_dictionary->GetString("name"));
    options.set_resolution(parameter_dictionary->GetDouble("resolution"));
    options.set_update_frequency(
        parameter_dictionary->GetDouble("update_frequency"));
    options.set_robot_radius(parameter_dictionary->GetDouble("robot_radius"));
    if (parameter_dictionary->HasKey("always_send_full_costmap")) {
        options.set_always_send_full_costmap(
            parameter_dictionary->GetBool("always_send_full_costmap"));
    }
    if (parameter_dictionary->HasKey("width")) {
        options.set_width(
            static_cast<int32_t>(parameter_dictionary->GetDouble("width")));
    }
    if (parameter_dictionary->HasKey("height")) {
        options.set_height(
            static_cast<int32_t>(parameter_dictionary->GetDouble("height")));
    }
    if (parameter_dictionary->HasKey("rolling_window")) {
        options.set_rolling_window(
            parameter_dictionary->GetBool("rolling_window"));
    }

    auto plugins_dict = parameter_dictionary->GetDictionary("plugins");
    auto plugins = plugins_dict->GetArrayValuesAsStrings();
    for (const auto& plugin : plugins) {
        options.add_plugins(plugin);
    }

    if (parameter_dictionary->HasKey("static_layer")) {
        auto static_layer_dict =
            parameter_dictionary->GetDictionary("static_layer");
        auto* static_layer = options.mutable_static_layer();
        static_layer->set_plugin(static_layer_dict->GetString("plugin"));
        static_layer->set_enabled(static_layer_dict->GetBool("enabled"));
        static_layer->set_subscribe_to_updates(
            static_layer_dict->GetBool("subscribe_to_updates"));
        static_layer->set_transform_tolerance(
            static_layer_dict->GetDouble("transform_tolerance"));
        static_layer->set_footprint_clearing_enabled(
            static_layer_dict->GetBool("footprint_clearing_enabled"));
        static_layer->set_map_topic(static_layer_dict->GetString("map_topic"));
    }

    if (parameter_dictionary->HasKey("denoise_layer")) {
        auto denoise_layer_dict =
            parameter_dictionary->GetDictionary("denoise_layer");
        auto* denoise_layer = options.mutable_denoise_layer();
        denoise_layer->set_plugin(denoise_layer_dict->GetString("plugin"));
        denoise_layer->set_enabled(denoise_layer_dict->GetBool("enabled"));
        denoise_layer->set_denoise_radius(
            denoise_layer_dict->GetDouble("denoise_radius"));
        if (denoise_layer_dict->HasKey("minimal_group_size")) {
            denoise_layer->set_denoise_radius(
                denoise_layer_dict->GetDouble("minimal_group_size"));
        }
        if (denoise_layer_dict->HasKey("group_connectivity_type")) {
            denoise_layer->set_group_connectivity_type(static_cast<int32_t>(
                denoise_layer_dict->GetInt("group_connectivity_type")));
        }
    }

    if (parameter_dictionary->HasKey("obstacle_layer")) {
        auto obstacle_layer_dict =
            parameter_dictionary->GetNonReferenceCountedDictionary(
                "obstacle_layer");
        auto* obstacle_layer = options.mutable_obstacle_layer();
        obstacle_layer->set_plugin(obstacle_layer_dict->GetString("plugin"));
        obstacle_layer->set_enabled(obstacle_layer_dict->GetBool("enabled"));
        obstacle_layer->set_footprint_clearing_enabled(
            obstacle_layer_dict->GetBool("footprint_clearing_enabled"));

        if (obstacle_layer_dict->HasKey("sensor_sources")) {
            auto sensor_sources_dict =
                obstacle_layer_dict->GetNonReferenceCountedDictionary(
                    "sensor_sources");
            for (const auto& source_name : sensor_sources_dict->GetKeys()) {
                auto source_dict =
                    sensor_sources_dict->GetDictionary(source_name);
                auto& sensor =
                    (*obstacle_layer->mutable_sensor_sources())[source_name];
                if (source_dict->HasKey("topic")) {
                    sensor.set_topic(source_dict->GetString("topic"));
                }
                if (source_dict->HasKey("data_type")) {
                    sensor.set_data_type(source_dict->GetString("data_type"));
                }
                if (source_dict->HasKey("max_obstacle_height")) {
                    sensor.set_max_obstacle_height(
                        source_dict->GetDouble("max_obstacle_height"));
                }
                if (source_dict->HasKey("min_obstacle_height")) {
                    sensor.set_min_obstacle_height(
                        source_dict->GetDouble("min_obstacle_height"));
                }
                if (source_dict->HasKey("obstacle_max_height")) {
                    sensor.set_obstacle_max_height(
                        source_dict->GetDouble("obstacle_max_height"));
                }
                if (source_dict->HasKey("obstacle_min_height")) {
                    sensor.set_obstacle_min_height(
                        source_dict->GetDouble("obstacle_min_height"));
                }
                if (source_dict->HasKey("marking")) {
                    sensor.set_marking(source_dict->GetBool("marking"));
                }
                if (source_dict->HasKey("clearing")) {
                    sensor.set_clearing(source_dict->GetBool("clearing"));
                }
                if (source_dict->HasKey("raytrace_max_range")) {
                    sensor.set_raytrace_max_range(
                        source_dict->GetDouble("raytrace_max_range"));
                }
                if (source_dict->HasKey("raytrace_min_range")) {
                    sensor.set_raytrace_min_range(
                        source_dict->GetDouble("raytrace_min_range"));
                }
            }
        }
    }

    if (parameter_dictionary->HasKey("voxel_layer")) {
        auto voxel_layer_dict =
            parameter_dictionary->GetNonReferenceCountedDictionary("voxel_layer");
        auto* voxel_layer = options.mutable_voxel_layer();
        voxel_layer->set_plugin(voxel_layer_dict->GetString("plugin"));
        voxel_layer->set_enabled(voxel_layer_dict->GetBool("enabled"));
        if (voxel_layer_dict->HasKey("footprint_clearing_enabled")) {
            voxel_layer->set_footprint_clearing_enabled(
                voxel_layer_dict->GetBool("footprint_clearing_enabled"));
        }
        if (voxel_layer_dict->HasKey("z_voxels")) {
            voxel_layer->set_z_voxels(
                static_cast<int32_t>(voxel_layer_dict->GetInt("z_voxels")));
        }
        if (voxel_layer_dict->HasKey("origin_z")) {
            voxel_layer->set_origin_z(voxel_layer_dict->GetDouble("origin_z"));
        }
        if (voxel_layer_dict->HasKey("z_resolution")) {
            voxel_layer->set_z_resolution(
                voxel_layer_dict->GetDouble("z_resolution"));
        }
        if (voxel_layer_dict->HasKey("max_obstacle_height")) {
            voxel_layer->set_max_obstacle_height(
                voxel_layer_dict->GetDouble("max_obstacle_height"));
        }
        if (voxel_layer_dict->HasKey("unknown_threshold")) {
            voxel_layer->set_unknown_threshold(
                static_cast<int32_t>(
                    voxel_layer_dict->GetInt("unknown_threshold")));
        }
        if (voxel_layer_dict->HasKey("mark_threshold")) {
            voxel_layer->set_mark_threshold(
                static_cast<int32_t>(voxel_layer_dict->GetInt("mark_threshold")));
        }
        if (voxel_layer_dict->HasKey("publish_voxel_map")) {
            voxel_layer->set_publish_voxel_map(
                voxel_layer_dict->GetBool("publish_voxel_map"));
        }
        if (voxel_layer_dict->HasKey("sensor_sources")) {
            auto sensor_sources_dict =
                voxel_layer_dict->GetNonReferenceCountedDictionary(
                    "sensor_sources");
            for (const auto& source_name : sensor_sources_dict->GetKeys()) {
                auto source_dict =
                    sensor_sources_dict->GetDictionary(source_name);
                auto& sensor =
                    (*voxel_layer->mutable_sensor_sources())[source_name];
                if (source_dict->HasKey("topic")) {
                    sensor.set_topic(source_dict->GetString("topic"));
                }
                if (source_dict->HasKey("data_type")) {
                    sensor.set_data_type(source_dict->GetString("data_type"));
                }
                if (source_dict->HasKey("max_obstacle_height")) {
                    sensor.set_max_obstacle_height(
                        source_dict->GetDouble("max_obstacle_height"));
                }
                if (source_dict->HasKey("min_obstacle_height")) {
                    sensor.set_min_obstacle_height(
                        source_dict->GetDouble("min_obstacle_height"));
                }
                if (source_dict->HasKey("marking")) {
                    sensor.set_marking(source_dict->GetBool("marking"));
                }
                if (source_dict->HasKey("clearing")) {
                    sensor.set_clearing(source_dict->GetBool("clearing"));
                }
                if (source_dict->HasKey("raytrace_max_range")) {
                    sensor.set_raytrace_max_range(
                        source_dict->GetDouble("raytrace_max_range"));
                }
                if (source_dict->HasKey("raytrace_min_range")) {
                    sensor.set_raytrace_min_range(
                        source_dict->GetDouble("raytrace_min_range"));
                }
            }
        }
    }

    if (parameter_dictionary->HasKey("range_sensor_layer")) {
        auto range_layer_dict =
            parameter_dictionary->GetNonReferenceCountedDictionary(
                "range_sensor_layer");
        auto* range_layer = options.mutable_range_sensor_layer();
        range_layer->set_plugin(range_layer_dict->GetString("plugin"));
        range_layer->set_enabled(range_layer_dict->GetBool("enabled"));
        if (range_layer_dict->HasKey("phi")) {
            range_layer->set_phi(range_layer_dict->GetDouble("phi"));
        }
        if (range_layer_dict->HasKey("inflate_cone")) {
            range_layer->set_inflate_cone(
                range_layer_dict->GetDouble("inflate_cone"));
        }
        if (range_layer_dict->HasKey("no_readings_timeout")) {
            range_layer->set_no_readings_timeout(
                range_layer_dict->GetDouble("no_readings_timeout"));
        }
        if (range_layer_dict->HasKey("clear_threshold")) {
            range_layer->set_clear_threshold(
                range_layer_dict->GetDouble("clear_threshold"));
        }
        if (range_layer_dict->HasKey("mark_threshold")) {
            range_layer->set_mark_threshold(
                range_layer_dict->GetDouble("mark_threshold"));
        }
        if (range_layer_dict->HasKey("clear_on_max_reading")) {
            range_layer->set_clear_on_max_reading(
                range_layer_dict->GetBool("clear_on_max_reading"));
        }
        if (range_layer_dict->HasKey("transform_tolerance")) {
            range_layer->set_transform_tolerance(
                range_layer_dict->GetDouble("transform_tolerance"));
        }
        if (range_layer_dict->HasKey("input_sensor_type")) {
            range_layer->set_input_sensor_type(
                range_layer_dict->GetString("input_sensor_type"));
        }
        if (range_layer_dict->HasKey("topics")) {
            auto topics_dict = range_layer_dict->GetDictionary("topics");
            for (const auto& topic : topics_dict->GetArrayValuesAsStrings()) {
                range_layer->add_topics(topic);
            }
        }
    }

    if (parameter_dictionary->HasKey("inflation_layer")) {
        auto inflation_layer_dict =
            parameter_dictionary->GetDictionary("inflation_layer");
        auto* inflation_layer = options.mutable_inflation_layer();
        inflation_layer->set_plugin(inflation_layer_dict->GetString("plugin"));
        inflation_layer->set_enabled(inflation_layer_dict->GetBool("enabled"));
        inflation_layer->set_cost_scaling_factor(
            inflation_layer_dict->GetDouble("cost_scaling_factor"));
        inflation_layer->set_inflation_radius(
            inflation_layer_dict->GetDouble("inflation_radius"));
        inflation_layer->set_inflate_unknown(
            inflation_layer_dict->GetBool("inflate_unknown"));
        inflation_layer->set_inflate_around_unknown(
            inflation_layer_dict->GetBool("inflate_around_unknown"));
    }

    return options;
}

}  // namespace map
}  // namespace autonomy
