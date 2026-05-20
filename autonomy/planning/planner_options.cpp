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

#include "autonomy/planning/planner_options.hpp"

#include "autonomy/common/logging.hpp"
#include "autonomy/common/config.hpp"
#include "autonomy/common/configuration_file_resolver.hpp"
#include "autonomy/common/lua_parameter_dictionary.hpp"
#include "autonomy/map/map_options.hpp"

#include <functional>

namespace autonomy {
namespace planning {

namespace {

proto::NavFnPlanner LoadNavFnPlannerOptions(
    autonomy::common::LuaParameterDictionary* const parameter_dictionary) {
    proto::NavFnPlanner options;
    options.set_tolerance(parameter_dictionary->GetDouble("tolerance"));
    options.set_use_astar(parameter_dictionary->GetBool("use_astar"));
    options.set_allow_unknown(parameter_dictionary->GetBool("allow_unknown"));
    options.set_use_final_approach_orientation(
        parameter_dictionary->GetBool("use_final_approach_orientation"));
    return options;
}

proto::DijkstraPlanner LoadDijkstraPlannerOptions(
    autonomy::common::LuaParameterDictionary* const parameter_dictionary) {
    proto::DijkstraPlanner options;
    options.set_tolerance(parameter_dictionary->GetDouble("tolerance"));
    options.set_allow_unknown(parameter_dictionary->GetBool("allow_unknown"));
    options.set_use_final_approach_orientation(
        parameter_dictionary->GetBool("use_final_approach_orientation"));
    return options;
}

proto::ThetaStarPlanner LoadThetaStarPlannerOptions(
    autonomy::common::LuaParameterDictionary* const parameter_dictionary) {
    proto::ThetaStarPlanner options;
    if (parameter_dictionary->HasKey("how_many_corners")) {
        options.set_how_many_corners(static_cast<int32_t>(
            parameter_dictionary->GetDouble("how_many_corners")));
    }
    if (parameter_dictionary->HasKey("allow_unknown")) {
        options.set_allow_unknown(parameter_dictionary->GetBool("allow_unknown"));
    }
    if (parameter_dictionary->HasKey("w_euc_cost")) {
        options.set_w_euc_cost(parameter_dictionary->GetDouble("w_euc_cost"));
    }
    if (parameter_dictionary->HasKey("w_traversal_cost")) {
        options.set_w_traversal_cost(
            parameter_dictionary->GetDouble("w_traversal_cost"));
    }
    if (parameter_dictionary->HasKey("w_heuristic_cost")) {
        options.set_w_heuristic_cost(
            parameter_dictionary->GetDouble("w_heuristic_cost"));
    }
    if (parameter_dictionary->HasKey("terminal_checking_interval")) {
        options.set_terminal_checking_interval(static_cast<int32_t>(
            parameter_dictionary->GetDouble("terminal_checking_interval")));
    }
    return options;
}

proto::SimpleSmootherOptions LoadSimpleSmootherOptions(
    autonomy::common::LuaParameterDictionary* const parameter_dictionary) {
    proto::SimpleSmootherOptions options;
    options.set_tolerance(parameter_dictionary->GetDouble("tolerance"));
    options.set_max_iterations(
        static_cast<int32_t>(parameter_dictionary->GetDouble("max_iterations")));
    options.set_w_data(parameter_dictionary->GetDouble("w_data"));
    options.set_w_smooth(parameter_dictionary->GetDouble("w_smooth"));
    options.set_do_refinement(parameter_dictionary->GetBool("do_refinement"));
    options.set_refinement_num(static_cast<int32_t>(
        parameter_dictionary->GetDouble("refinement_num")));
    options.set_enforce_path_inversion(
        parameter_dictionary->GetBool("enforce_path_inversion"));
    return options;
}

proto::SavitzkyGolaySmootherOptions LoadSavitzkyGolaySmootherOptions(
    autonomy::common::LuaParameterDictionary* const parameter_dictionary) {
    proto::SavitzkyGolaySmootherOptions options;
    options.set_do_refinement(parameter_dictionary->GetBool("do_refinement"));
    options.set_refinement_num(static_cast<int32_t>(
        parameter_dictionary->GetDouble("refinement_num")));
    options.set_enforce_path_inversion(
        parameter_dictionary->GetBool("enforce_path_inversion"));
    options.set_window_size(
        static_cast<int32_t>(parameter_dictionary->GetDouble("window_size")));
    options.set_poly_order(
        static_cast<int32_t>(parameter_dictionary->GetDouble("poly_order")));
    return options;
}

proto::math::FemPosDeviationSmootherConfig LoadFemPosSmootherOptions(
    autonomy::common::LuaParameterDictionary* const parameter_dictionary) {
    proto::math::FemPosDeviationSmootherConfig options;
    if (parameter_dictionary->HasKey("weight_fem_pos_deviation")) {
        options.set_weight_fem_pos_deviation(
            parameter_dictionary->GetDouble("weight_fem_pos_deviation"));
    }
    if (parameter_dictionary->HasKey("weight_ref_deviation")) {
        options.set_weight_ref_deviation(
            parameter_dictionary->GetDouble("weight_ref_deviation"));
    }
    if (parameter_dictionary->HasKey("weight_path_length")) {
        options.set_weight_path_length(
            parameter_dictionary->GetDouble("weight_path_length"));
    }
    if (parameter_dictionary->HasKey("max_iter")) {
        options.set_max_iter(static_cast<int32_t>(
            parameter_dictionary->GetDouble("max_iter")));
    }
    if (parameter_dictionary->HasKey("time_limit")) {
        options.set_time_limit(parameter_dictionary->GetDouble("time_limit"));
    }
    return options;
}

proto::math::CosThetaSmootherConfig LoadCosThetaSmootherOptions(
    autonomy::common::LuaParameterDictionary* const parameter_dictionary) {
    proto::math::CosThetaSmootherConfig options;
    if (parameter_dictionary->HasKey("weight_cos_included_angle")) {
        options.set_weight_cos_included_angle(
            parameter_dictionary->GetDouble("weight_cos_included_angle"));
    }
    if (parameter_dictionary->HasKey("weight_anchor_points")) {
        options.set_weight_anchor_points(
            parameter_dictionary->GetDouble("weight_anchor_points"));
    }
    if (parameter_dictionary->HasKey("weight_length")) {
        options.set_weight_length(
            parameter_dictionary->GetDouble("weight_length"));
    }
    return options;
}

void LoadStringArray(
    autonomy::common::LuaParameterDictionary* dict,
    const std::function<void(const std::string&)>& add_fn) {
    const auto values = dict->GetArrayValuesAsStrings();
    for (const auto& value : values) {
        add_fn(value);
    }
}

}  // namespace

proto::PlannerOptions LoadOptions(
    autonomy::common::LuaParameterDictionary* const parameter_dictionary) {
    proto::PlannerOptions options;
    options.set_expected_planner_frequency(
        parameter_dictionary->GetDouble("expected_planner_frequency"));
    options.set_costmap_update_timeout(
        parameter_dictionary->GetDouble("costmap_update_timeout"));
    *options.mutable_navfn() = LoadNavFnPlannerOptions(
        parameter_dictionary->GetDictionary("navfn_planner").get());
    if (parameter_dictionary->HasKey("dijkstra_planner")) {
        *options.mutable_dijkstra() = LoadDijkstraPlannerOptions(
            parameter_dictionary->GetDictionary("dijkstra_planner").get());
    }
    if (parameter_dictionary->HasKey("theta_star_planner")) {
        *options.mutable_theta_star() = LoadThetaStarPlannerOptions(
            parameter_dictionary->GetDictionary("theta_star_planner").get());
    }
    *options.mutable_costmap() = map::CreateCostmap2DOptions(
        parameter_dictionary->GetDictionary("costmap").get());

    if (parameter_dictionary->HasKey("planner_plugins")) {
        LoadStringArray(
            parameter_dictionary->GetDictionary("planner_plugins").get(),
            [&options](const std::string& v) { options.add_planner_plugins(v); });
    }
    if (parameter_dictionary->HasKey("planner_plugin_libraries")) {
        LoadStringArray(
            parameter_dictionary->GetDictionary("planner_plugin_libraries")
                .get(),
            [&options](const std::string& v) {
                options.add_planner_plugin_libraries(v);
            });
    }
    if (parameter_dictionary->HasKey("default_planner_id")) {
        options.set_default_planner_id(
            parameter_dictionary->GetString("default_planner_id"));
    }

    if (parameter_dictionary->HasKey("smoother_plugins")) {
        LoadStringArray(
            parameter_dictionary->GetDictionary("smoother_plugins").get(),
            [&options](const std::string& v) { options.add_smoother_plugins(v); });
    }
    if (parameter_dictionary->HasKey("default_smoother_id")) {
        options.set_default_smoother_id(
            parameter_dictionary->GetString("default_smoother_id"));
    }
    if (parameter_dictionary->HasKey("simple_smoother")) {
        *options.mutable_simple_smoother() = LoadSimpleSmootherOptions(
            parameter_dictionary->GetDictionary("simple_smoother").get());
    }
    if (parameter_dictionary->HasKey("savitzky_golay_smoother")) {
        *options.mutable_savitzky_golay_smoother() =
            LoadSavitzkyGolaySmootherOptions(
                parameter_dictionary->GetDictionary("savitzky_golay_smoother")
                    .get());
    }
    if (parameter_dictionary->HasKey("fem_pos_smoother")) {
        *options.mutable_fem_pos_smoother() = LoadFemPosSmootherOptions(
            parameter_dictionary->GetDictionary("fem_pos_smoother").get());
    }
    if (parameter_dictionary->HasKey("fem_pos_smoother_path_bound")) {
        options.set_fem_pos_smoother_path_bound(
            parameter_dictionary->GetDouble("fem_pos_smoother_path_bound"));
    }
    if (parameter_dictionary->HasKey("cos_theta_smoother")) {
        *options.mutable_cos_theta_smoother() = LoadCosThetaSmootherOptions(
            parameter_dictionary->GetDictionary("cos_theta_smoother").get());
    }
    if (parameter_dictionary->HasKey("cos_theta_smoother_path_bound")) {
        options.set_cos_theta_smoother_path_bound(
            parameter_dictionary->GetDouble("cos_theta_smoother_path_bound"));
    }
    if (parameter_dictionary->HasKey("path_simplify_epsilon")) {
        options.set_path_simplify_epsilon(
            parameter_dictionary->GetDouble("path_simplify_epsilon"));
    }
    if (parameter_dictionary->HasKey("auto_smooth_after_plan")) {
        options.set_auto_smooth_after_plan(
            parameter_dictionary->GetBool("auto_smooth_after_plan"));
    }
    if (parameter_dictionary->HasKey("auto_smooth_duration")) {
        options.set_auto_smooth_duration(
            parameter_dictionary->GetDouble("auto_smooth_duration"));
    }

    return options;
}

}  // namespace planning
}  // namespace autonomy
