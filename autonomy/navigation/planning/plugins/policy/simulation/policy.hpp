/*
 * Copyright 2026 The Navigation Authors (navigation)
 */

/**
 * @file policy.hpp
 * @brief Simulation policy: pose goal seek + map frontier exploration.
 */

#pragma once

#include <string_view>

#include "navigation/planning/interface/policy_interface.hpp"
#include "navigation/proto/plugins.pb.h"

namespace navigation
{
namespace planning
{
namespace simulation
{

/**
 * @brief Dependency-free policy used by autosim integration.
 *
 * @details Handles TYPE_POSE by proportional goal seeking and TYPE_EXPLORATION
 * by selecting the nearest frontier from Observation::map.
 */
class Policy final : public PolicyInterface
{
public:
    /** @brief Construct with VINT-compatible options. */
    explicit Policy(plugins::VintPolicyOptions options);

    /** @brief Accept the "simulation" plugin name. */
    bool Configure(const config::PolicyOptions& options) override;

    /** @brief Produce a velocity intent for pose or exploration goals. */
    common::StatusOr<NavigationIntent> Query(
        const observation::Observation& observation,
        const observation::DepthFeature& depth_feature, const GoalSpec& goal,
        const memory::ContextView& context) override;

    /** @brief Reset cached state. */
    void Reset() override;

    /** @brief Plugin name ("simulation"). */
    [[nodiscard]] std::string_view Name() const override;

private:
    /** @brief True after Configure succeeds. */
    bool configured_{false};
};

}  // namespace simulation
}  // namespace planning
}  // namespace navigation
