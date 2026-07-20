/*
 * Copyright 2026 The Openbot Authors
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

#include "autonomy/exploration/exploration_options.hpp"

#include <string>

#include "autonomy/common/configuration_file_resolver.hpp"
#include "autonomy/common/lua_parameter_dictionary.hpp"

namespace autonomy {
namespace exploration {
namespace {

void LoadCamera(
    ::autonomy::common::LuaParameterDictionary* dict,
    proto::CameraFoVOptions* cam)
{
    if (!dict || !cam) {
        return;
    }
    if (dict->HasKey("hfov_deg")) {
        cam->set_hfov_deg(dict->GetDouble("hfov_deg"));
    }
    if (dict->HasKey("vfov_deg")) {
        cam->set_vfov_deg(dict->GetDouble("vfov_deg"));
    }
    if (dict->HasKey("z_near")) {
        cam->set_z_near(dict->GetDouble("z_near"));
    }
    if (dict->HasKey("z_far")) {
        cam->set_z_far(dict->GetDouble("z_far"));
    }
    if (dict->HasKey("fx")) {
        cam->set_fx(dict->GetDouble("fx"));
    }
    if (dict->HasKey("fy")) {
        cam->set_fy(dict->GetDouble("fy"));
    }
    if (dict->HasKey("cx")) {
        cam->set_cx(dict->GetDouble("cx"));
    }
    if (dict->HasKey("cy")) {
        cam->set_cy(dict->GetDouble("cy"));
    }
}

}  // namespace

proto::ExplorationOptions DefaultOptions()
{
    proto::ExplorationOptions options;
    auto* cam = options.mutable_camera();
    cam->set_hfov_deg(87.0);
    cam->set_vfov_deg(58.0);
    cam->set_z_near(0.3);
    cam->set_z_far(5.0);

    auto* occ = options.mutable_occupancy();
    occ->set_resolution(0.25);
    occ->set_size_x(120);
    occ->set_size_y(120);
    occ->set_size_z(1);
    occ->set_fuse_z_min(-0.5);
    occ->set_fuse_z_max(1.5);
    occ->set_log_odds_hit(0.85);
    occ->set_log_odds_miss(-0.4);
    occ->set_log_odds_clamp(3.5);
    occ->set_log_odds_occupy_threshold(0.85);
    occ->set_log_odds_free_threshold(-0.4);

    auto* vp = options.mutable_viewpoint();
    vp->set_number_x(40);
    vp->set_number_y(40);
    vp->set_resolution(0.6);
    vp->set_yaw_samples(8);
    vp->set_min_gain(0.3);

    auto* gw = options.mutable_grid_world();
    gw->set_cell_size(4.0);
    gw->set_nearby_cell_radius(2);
    gw->set_covered_gain_threshold(2.0);
    gw->set_exploration_area_half_extent(10.0);

    options.set_lookahead_distance(2.0);
    options.set_keypose_min_dist(1.0);
    options.set_collision_radius(0.35);
    options.set_use_frontier(true);
    options.set_depth_subsample(4);
    options.set_planner_frequency(1.0);
    options.set_tsp_max_exact_n(12);
    options.set_local_tsp_max_nodes(10);
    options.set_los_stop_at_unknown(false);
    options.set_occlusion_enabled(true);
    options.set_frontier_cluster_dist(0.75);
    options.set_replan_min_period_sec(1.0);
    options.set_keypose_astar_detour_ratio(2.5);
    options.add_explorer_plugins("rgbd_hierarchical");
    options.set_default_explorer("rgbd_hierarchical");
    return options;
}

proto::ExplorationOptions LoadOptions(
    ::autonomy::common::LuaParameterDictionary* const parameter_dictionary)
{
    proto::ExplorationOptions options = DefaultOptions();
    if (!parameter_dictionary) {
        return options;
    }

    if (parameter_dictionary->HasKey("camera")) {
        LoadCamera(parameter_dictionary->GetDictionary("camera").get(),
                   options.mutable_camera());
    }
    if (parameter_dictionary->HasKey("occupancy")) {
        auto dict = parameter_dictionary->GetDictionary("occupancy");
        auto* occ = options.mutable_occupancy();
        if (dict->HasKey("resolution")) {
            occ->set_resolution(dict->GetDouble("resolution"));
        }
        if (dict->HasKey("size_x")) {
            occ->set_size_x(static_cast<int>(dict->GetDouble("size_x")));
        }
        if (dict->HasKey("size_y")) {
            occ->set_size_y(static_cast<int>(dict->GetDouble("size_y")));
        }
        if (dict->HasKey("size_z")) {
            occ->set_size_z(static_cast<int>(dict->GetDouble("size_z")));
        }
        if (dict->HasKey("fuse_z_min")) {
            occ->set_fuse_z_min(dict->GetDouble("fuse_z_min"));
        }
        if (dict->HasKey("fuse_z_max")) {
            occ->set_fuse_z_max(dict->GetDouble("fuse_z_max"));
        }
        if (dict->HasKey("log_odds_hit")) {
            occ->set_log_odds_hit(dict->GetDouble("log_odds_hit"));
        }
        if (dict->HasKey("log_odds_miss")) {
            occ->set_log_odds_miss(dict->GetDouble("log_odds_miss"));
        }
        if (dict->HasKey("log_odds_clamp")) {
            occ->set_log_odds_clamp(dict->GetDouble("log_odds_clamp"));
        }
        if (dict->HasKey("log_odds_occupy_threshold")) {
            occ->set_log_odds_occupy_threshold(
                dict->GetDouble("log_odds_occupy_threshold"));
        }
        if (dict->HasKey("log_odds_free_threshold")) {
            occ->set_log_odds_free_threshold(
                dict->GetDouble("log_odds_free_threshold"));
        }
    }
    if (parameter_dictionary->HasKey("viewpoint")) {
        auto dict = parameter_dictionary->GetDictionary("viewpoint");
        auto* vp = options.mutable_viewpoint();
        if (dict->HasKey("number_x")) {
            vp->set_number_x(static_cast<int>(dict->GetDouble("number_x")));
        }
        if (dict->HasKey("number_y")) {
            vp->set_number_y(static_cast<int>(dict->GetDouble("number_y")));
        }
        if (dict->HasKey("resolution")) {
            vp->set_resolution(dict->GetDouble("resolution"));
        }
        if (dict->HasKey("yaw_samples")) {
            vp->set_yaw_samples(
                static_cast<int>(dict->GetDouble("yaw_samples")));
        }
        if (dict->HasKey("robot_height")) {
            vp->set_robot_height(dict->GetDouble("robot_height"));
        }
        if (dict->HasKey("min_gain")) {
            vp->set_min_gain(dict->GetDouble("min_gain"));
        }
    }
    if (parameter_dictionary->HasKey("grid_world")) {
        auto dict = parameter_dictionary->GetDictionary("grid_world");
        auto* gw = options.mutable_grid_world();
        if (dict->HasKey("cell_size")) {
            gw->set_cell_size(dict->GetDouble("cell_size"));
        }
        if (dict->HasKey("nearby_cell_radius")) {
            gw->set_nearby_cell_radius(
                static_cast<int>(dict->GetDouble("nearby_cell_radius")));
        }
        if (dict->HasKey("covered_gain_threshold")) {
            gw->set_covered_gain_threshold(
                dict->GetDouble("covered_gain_threshold"));
        }
        if (dict->HasKey("exploration_area_half_extent")) {
            gw->set_exploration_area_half_extent(
                dict->GetDouble("exploration_area_half_extent"));
        }
    }
    if (parameter_dictionary->HasKey("lookahead_distance")) {
        options.set_lookahead_distance(
            parameter_dictionary->GetDouble("lookahead_distance"));
    }
    if (parameter_dictionary->HasKey("keypose_min_dist")) {
        options.set_keypose_min_dist(
            parameter_dictionary->GetDouble("keypose_min_dist"));
    }
    if (parameter_dictionary->HasKey("collision_radius")) {
        options.set_collision_radius(
            parameter_dictionary->GetDouble("collision_radius"));
    }
    if (parameter_dictionary->HasKey("use_frontier")) {
        options.set_use_frontier(
            parameter_dictionary->GetBool("use_frontier"));
    }
    if (parameter_dictionary->HasKey("depth_subsample")) {
        options.set_depth_subsample(
            static_cast<int>(parameter_dictionary->GetDouble("depth_subsample")));
    }
    if (parameter_dictionary->HasKey("planner_frequency")) {
        options.set_planner_frequency(
            parameter_dictionary->GetDouble("planner_frequency"));
    }
    if (parameter_dictionary->HasKey("tsp_max_exact_n")) {
        options.set_tsp_max_exact_n(static_cast<int>(
            parameter_dictionary->GetDouble("tsp_max_exact_n")));
    }
    if (parameter_dictionary->HasKey("local_tsp_max_nodes")) {
        options.set_local_tsp_max_nodes(static_cast<int>(
            parameter_dictionary->GetDouble("local_tsp_max_nodes")));
    }
    if (parameter_dictionary->HasKey("los_stop_at_unknown")) {
        options.set_los_stop_at_unknown(
            parameter_dictionary->GetBool("los_stop_at_unknown"));
    }
    if (parameter_dictionary->HasKey("occlusion_enabled")) {
        options.set_occlusion_enabled(
            parameter_dictionary->GetBool("occlusion_enabled"));
    }
    if (parameter_dictionary->HasKey("frontier_cluster_dist")) {
        options.set_frontier_cluster_dist(
            parameter_dictionary->GetDouble("frontier_cluster_dist"));
    }
    if (parameter_dictionary->HasKey("replan_min_period_sec")) {
        options.set_replan_min_period_sec(
            parameter_dictionary->GetDouble("replan_min_period_sec"));
    }
    if (parameter_dictionary->HasKey("keypose_astar_detour_ratio")) {
        options.set_keypose_astar_detour_ratio(
            parameter_dictionary->GetDouble("keypose_astar_detour_ratio"));
    }
    if (parameter_dictionary->HasKey("default_explorer")) {
        options.set_default_explorer(
            parameter_dictionary->GetString("default_explorer"));
    }
    if (parameter_dictionary->HasKey("explorer_plugins")) {
        options.clear_explorer_plugins();
        const auto values =
            parameter_dictionary->GetDictionary("explorer_plugins")
                ->GetArrayValuesAsStrings();
        for (const auto& v : values) {
            options.add_explorer_plugins(v);
        }
    }
    return options;
}

proto::ExplorationOptions CreateOptions(
    const std::string& configuration_directory)
{
    auto file_resolver =
        std::make_unique<::autonomy::common::ConfigurationFileResolver>(
            std::vector<std::string>{configuration_directory});
    if (!file_resolver) {
        return DefaultOptions();
    }
    try {
        const std::string code =
            file_resolver->GetFileContentOrDie("exploration.lua");
        ::autonomy::common::LuaParameterDictionary dict(
            code, std::move(file_resolver));
        return LoadOptions(&dict);
    } catch (...) {
        return DefaultOptions();
    }
}

}  // namespace exploration
}  // namespace autonomy
