/* Copyright 2026 The Openbot Authors. */
#pragma once
#include "autonomy/task/exploration/viewpoint/viewpoint_evaluator.hpp"
#include "autonomy/task/exploration/region/region_manager.hpp"
#include "opt/navigation/reachability.h"
#include <functional>
namespace autonomy::task::exploration {
struct ViewpointSamplingOptions {double spacing=.6,local_radius=5,nominal_speed=.4,angular_speed=.5,observation_time=.5;unsigned yaw_samples=8;std::size_t maximum_candidates=512,maximum_evaluations=8192;};
struct ViewpointSamplingResult {
    std::vector<Viewpoint> candidates;
    bool exhaustive=false,limited=false,finished=false,canceled=false;
    std::size_t next_cursor=0;
};
class ViewpointSampler {
public:
    ViewpointSamplingResult Sample(const Pose2d& start,const MapReadView& map,const CoverageState& coverage,
        const ViewpointEvaluator& evaluator,const RegionManager& regions,perception::SensorMask active,
        opt::navigation::SafetyOptions safety,ViewpointSamplingOptions options={},bool sweep=false,
        std::size_t cursor=0,const std::function<bool()>& cancel_checker={}) const;
};
}  // namespace autonomy::task::exploration
