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
    options.set_frame_id(parameter_dictionary->GetString("frame_id"));
    options.set_name(parameter_dictionary->GetString("name"));
    options.set_resolution(parameter_dictionary->GetDouble("resolution"));
    options.set_update_frequency(
        parameter_dictionary->GetDouble("update_frequency"));
    options.set_robot_radius(parameter_dictionary->GetDouble("robot_radius"));
    options.set_always_send_full_costmap(
        parameter_dictionary->GetBool("always_send_full_costmap"));

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
