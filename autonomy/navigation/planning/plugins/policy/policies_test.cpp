/* Copyright 2026 The Navigation Authors (navigation) */
#include <cmath>
#include <iostream>

#include "navigation/planning/plugins/policy/nomad/policy.hpp"
#include "navigation/planning/plugins/policy/simulation/policy.hpp"
#include "navigation/planning/plugins/policy/vint/policy.hpp"

namespace {

navigation::GoalSpec Goal() {
    navigation::GoalSpec goal;
    goal.set_type(navigation::GoalSpec::TYPE_POSE);
    goal.mutable_pose()->mutable_pose()->mutable_position()->set_x(2.0);
    goal.mutable_pose()->mutable_pose()->mutable_orientation()->set_w(1.0);
    return goal;
}

}  // namespace

int main() {
    using namespace navigation;  // NOLINT

    plugins::VintPolicyOptions vint_options;
    vint_options.set_action_chunk_length(8);
    planning::vint::Policy vint(vint_options);
    config::PolicyOptions policy_options;
    policy_options.set_name("vint");
    *policy_options.mutable_vint() = vint_options;
    if (!vint.Configure(policy_options)) {
        std::cerr << "vint configure failed\n";
        return 1;
    }
    observation::Observation observation;
    observation.mutable_odom()
        ->mutable_pose()
        ->mutable_pose()
        ->mutable_pose()
        ->mutable_orientation()
        ->set_w(1.0);
    observation::DepthFeature feature;
    const auto vint_result =
        vint.Query(observation, feature, Goal(), memory::ContextView{});
    if (!vint_result.Ok() || !vint_result.Value().has_velocity() ||
        vint_result.Value().velocity().x() <= 0.0) {
        std::cerr << "vint query failed\n";
        return 1;
    }

    plugins::NomadPolicyOptions nomad_options;
    nomad_options.set_action_chunk_length(8);
    planning::nomad::Policy nomad(nomad_options);
    config::PolicyOptions nomad_policy;
    nomad_policy.set_name("nomad");
    *nomad_policy.mutable_nomad() = nomad_options;
    if (!nomad.Configure(nomad_policy)) {
        std::cerr << "nomad configure failed\n";
        return 1;
    }
    const auto nomad_result =
        nomad.Query(observation, feature, Goal(), memory::ContextView{});
    if (!nomad_result.Ok() || !nomad_result.Value().has_chunk() ||
        nomad_result.Value().chunk().actions_size() != 8) {
        std::cerr << "nomad query failed\n";
        return 1;
    }
    planning::simulation::Policy simulation(vint_options);
    config::PolicyOptions simulation_options;
    simulation_options.set_name("simulation");
    if (!simulation.Configure(simulation_options))
        return 1;
    auto nearby = Goal();
    nearby.mutable_pose()->mutable_pose()->mutable_position()->set_x(0.175);
    const auto approaching =
        simulation.Query(observation, feature, nearby, memory::ContextView{});
    if (!approaching.Ok() ||
        approaching.Value().chunk().actions(0).x() <= 0.0) {
        std::cerr << "simulation stopped outside Runner position tolerance\n";
        return 1;
    }
    nearby.set_position_tolerance(0.2);
    nearby.mutable_pose()->mutable_pose()->mutable_orientation()->set_z(
        std::sin(0.25));
    nearby.mutable_pose()->mutable_pose()->mutable_orientation()->set_w(
        std::cos(0.25));
    const auto aligning =
        simulation.Query(observation, feature, nearby, memory::ContextView{});
    if (!aligning.Ok() || aligning.Value().chunk().actions(0).x() != 0.0 ||
        aligning.Value().chunk().actions(0).theta() <= 0.0 ||
        aligning.Value().chunk().actions(0).theta() > 0.800001) {
        std::cerr << "simulation did not align requested goal heading\n";
        return 1;
    }
    auto behind = Goal();
    behind.mutable_pose()->mutable_pose()->mutable_position()->set_x(-2.0);
    const auto turning =
        simulation.Query(observation, feature, behind, memory::ContextView{});
    if (!turning.Ok() || turning.Value().chunk().actions(0).x() != 0.0 ||
        std::abs(turning.Value().chunk().actions(0).theta()) < 0.1) {
        std::cerr << "simulation drove forward with target behind\n";
        return 1;
    }
    return 0;
}
