/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/control/controller/mppi_controller/parameter_options.hpp"

#include "autonomy/common/lua_parameter_dictionary.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace mppi {
namespace {

template <typename CriticMsg>
void LoadBaseCritic(::autonomy::common::LuaParameterDictionary* dict,
                    CriticMsg* critic) {
    if (!dict || !critic) {
        return;
    }
    if (dict->HasKey("enabled")) {
        critic->set_enabled(dict->GetBool("enabled"));
    }
    if (dict->HasKey("cost_power")) {
        critic->set_cost_power(static_cast<int32_t>(dict->GetDouble("cost_power")));
    }
    if (dict->HasKey("cost_weight")) {
        critic->set_cost_weight(dict->GetDouble("cost_weight"));
    }
}

}  // namespace

proto::MPPIControllerOptions LoadOptions(
    ::autonomy::common::LuaParameterDictionary* const parameter_dictionary) {
    proto::MPPIControllerOptions options;
    if (!parameter_dictionary) {
        return options;
    }

    auto load_double = [&](const char* key, auto setter) {
        if (parameter_dictionary->HasKey(key)) {
            setter(parameter_dictionary->GetDouble(key));
        }
    };
    auto load_string = [&](const char* key, auto setter) {
        if (parameter_dictionary->HasKey(key)) {
            setter(parameter_dictionary->GetString(key));
        }
    };
    auto load_bool = [&](const char* key, auto setter) {
        if (parameter_dictionary->HasKey(key)) {
            setter(parameter_dictionary->GetBool(key));
        }
    };
    auto load_int = [&](const char* key, auto setter) {
        if (parameter_dictionary->HasKey(key)) {
            setter(static_cast<int32_t>(parameter_dictionary->GetDouble(key)));
        }
    };

    load_int("time_steps", [&](int32_t v) { options.set_time_steps(v); });
    load_double("model_dt", [&](double v) { options.set_model_dt(v); });
    load_int("batch_size", [&](int32_t v) { options.set_batch_size(v); });
    load_double("vx_std", [&](double v) { options.set_vx_std(v); });
    load_double("vy_std", [&](double v) { options.set_vy_std(v); });
    load_double("wz_std", [&](double v) { options.set_wz_std(v); });
    load_double("vx_max", [&](double v) { options.set_vx_max(v); });
    load_double("vx_min", [&](double v) { options.set_vx_min(v); });
    load_double("vy_max", [&](double v) { options.set_vy_max(v); });
    load_double("wz_max", [&](double v) { options.set_wz_max(v); });
    load_int("iteration_count", [&](int32_t v) { options.set_iteration_count(v); });
    load_double("temperature", [&](double v) { options.set_temperature(v); });
    load_double("gamma", [&](double v) { options.set_gamma(v); });
    load_string("motion_model", [&](const std::string& v) {
        options.set_motion_model(v);
    });
    load_bool("visualize", [&](bool v) { options.set_visualize(v); });

    if (parameter_dictionary->HasKey("critics")) {
        auto critics_dict = parameter_dictionary->GetDictionary("critics");
        for (const auto& name : critics_dict->GetArrayValuesAsStrings()) {
            options.add_critics(name);
        }
    }

    if (parameter_dictionary->HasKey("TrajectoryVisualizer")) {
        auto tv = parameter_dictionary->GetDictionary("TrajectoryVisualizer");
        auto* vis = options.mutable_trajectory_visualizer();
        if (tv->HasKey("trajectory_step")) {
            vis->set_trajectory_step(
                static_cast<int32_t>(tv->GetDouble("trajectory_step")));
        }
        if (tv->HasKey("time_step")) {
            vis->set_time_step(static_cast<int32_t>(tv->GetDouble("time_step")));
        }
    }

    if (parameter_dictionary->HasKey("AckermannConstraints")) {
        auto ack = parameter_dictionary->GetDictionary("AckermannConstraints");
        if (ack->HasKey("min_turning_r")) {
            options.mutable_ackermann_constraints()->set_min_turning_r(
                ack->GetDouble("min_turning_r"));
        }
    }

    auto load_critic_block = [&](const char* key, auto fill) {
        if (parameter_dictionary->HasKey(key)) {
            fill(parameter_dictionary->GetDictionary(key).get());
        }
    };

    load_critic_block("ConstraintCritic", [&](auto* d) {
        LoadBaseCritic(d, options.mutable_constraint_critic());
    });
    load_critic_block("GoalCritic", [&](auto* d) {
        auto* c = options.mutable_goal_critic();
        LoadBaseCritic(d, c);
        if (d->HasKey("threshold_to_consider")) {
            c->set_threshold_to_consider(d->GetDouble("threshold_to_consider"));
        }
    });
    load_critic_block("GoalAngleCritic", [&](auto* d) {
        auto* c = options.mutable_goal_angle_critic();
        LoadBaseCritic(d, c);
        if (d->HasKey("threshold_to_consider")) {
            c->set_threshold_to_consider(d->GetDouble("threshold_to_consider"));
        }
    });
    load_critic_block("PreferForwardCritic", [&](auto* d) {
        auto* c = options.mutable_prefer_forward_critic();
        LoadBaseCritic(d, c);
        if (d->HasKey("threshold_to_consider")) {
            c->set_threshold_to_consider(d->GetDouble("threshold_to_consider"));
        }
    });
    load_critic_block("CostCritic", [&](auto* d) {
        auto* c = options.mutable_cost_critic();
        LoadBaseCritic(d, c);
        if (d->HasKey("critical_cost")) {
            c->set_critical_cost(d->GetDouble("critical_cost"));
        }
        if (d->HasKey("consider_footprint")) {
            c->set_consider_footprint(d->GetBool("consider_footprint"));
        }
        if (d->HasKey("collision_cost")) {
            c->set_collision_cost(d->GetDouble("collision_cost"));
        }
        if (d->HasKey("near_goal_distance")) {
            c->set_near_goal_distance(d->GetDouble("near_goal_distance"));
        }
        if (d->HasKey("trajectory_point_step")) {
            c->set_trajectory_point_step(
                static_cast<int32_t>(d->GetDouble("trajectory_point_step")));
        }
    });
    load_critic_block("PathAlignCritic", [&](auto* d) {
        auto* c = options.mutable_path_align_critic();
        LoadBaseCritic(d, c);
        if (d->HasKey("max_path_occupancy_ratio")) {
            c->set_max_path_occupancy_ratio(d->GetDouble("max_path_occupancy_ratio"));
        }
        if (d->HasKey("trajectory_point_step")) {
            c->set_trajectory_point_step(
                static_cast<int32_t>(d->GetDouble("trajectory_point_step")));
        }
        if (d->HasKey("threshold_to_consider")) {
            c->set_threshold_to_consider(d->GetDouble("threshold_to_consider"));
        }
        if (d->HasKey("offset_from_furthest")) {
            c->set_offset_from_furthest(
                static_cast<int32_t>(d->GetDouble("offset_from_furthest")));
        }
        if (d->HasKey("use_path_orientations")) {
            c->set_use_path_orientations(d->GetBool("use_path_orientations"));
        }
    });
    load_critic_block("PathFollowCritic", [&](auto* d) {
        auto* c = options.mutable_path_follow_critic();
        LoadBaseCritic(d, c);
        if (d->HasKey("offset_from_furthest")) {
            c->set_offset_from_furthest(
                static_cast<int32_t>(d->GetDouble("offset_from_furthest")));
        }
        if (d->HasKey("threshold_to_consider")) {
            c->set_threshold_to_consider(d->GetDouble("threshold_to_consider"));
        }
    });
    load_critic_block("PathAngleCritic", [&](auto* d) {
        auto* c = options.mutable_path_angle_critic();
        LoadBaseCritic(d, c);
        if (d->HasKey("offset_from_furthest")) {
            c->set_offset_from_furthest(
                static_cast<int32_t>(d->GetDouble("offset_from_furthest")));
        }
        if (d->HasKey("threshold_to_consider")) {
            c->set_threshold_to_consider(d->GetDouble("threshold_to_consider"));
        }
        if (d->HasKey("max_angle_to_furthest")) {
            c->set_max_angle_to_furthest(d->GetDouble("max_angle_to_furthest"));
        }
        if (d->HasKey("forward_preference")) {
            c->set_forward_preference(d->GetBool("forward_preference"));
        }
    });

    return options;
}

}  // namespace mppi
}  // namespace controller
}  // namespace control
}  // namespace autonomy
