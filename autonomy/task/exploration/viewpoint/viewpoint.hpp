/* Copyright 2026 The Openbot Authors. */
#pragma once
#include "autonomy/task/exploration/coverage/coverage_state.hpp"
#include "autonomy/task/exploration/integration/map_reader.hpp"
namespace autonomy::task::exploration {
struct Viewpoint {
    std::uint64_t id=0;
    Pose2d pose;
    std::pair<int,int> region_id;
    std::vector<CoverageContribution> contributions;
    double predicted_gain=0,travel_time=0,turn_time=0,observation_time=0;
    double Utility(double gain) const{return gain/(travel_time+turn_time+observation_time+1e-6);}
};
struct ObservationGoal {
    std::uint64_t goal_id=0,goal_version=0,map_epoch=0,coverage_revision=0;
    Pose2d original_pose;
    std::pair<int,int> region_id;
    perception::SensorMode sensor_mode=perception::SensorMode::kRgbdOnly;
    std::vector<CoverageId> required_coverage_units;
    std::vector<double> required_views;
    double allowed_translation=.3,allowed_yaw_change=.3,minimum_gain=0;
    // Completion is always actual observation confirmed, never arrival.
};
}  // namespace autonomy::task::exploration
