/*
 * Copyright 2026 The Navigation Authors (navigation)
 */

/**
 * @file policy.hpp
 * @brief Simulation policy: pose goal seek + frontier exploration.
 */

#pragma once

#include <chrono>
#include <memory>
#include <deque>
#include <functional>
#include <optional>
#include <string_view>
#include <vector>

#include "navigation/planning/base/waypoint.hpp"
#include "navigation/common/twist2d.hpp"
#include "navigation/planning/interface/policy_interface.hpp"
#include "navigation/planning/tasks/frontiernet_frontier_source.hpp"
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
 * by retaining reachable frontier goals with bounded history and timeouts.
 * Occupancy maps are authoritative; FrontierNet depth projections are used
 * when no map is available. No-frontier fallback performs an odometry scan.
 */
class Policy final : public PolicyInterface
{
public:
    /** @brief Construct with VINT-compatible options. */
    explicit Policy(plugins::VintPolicyOptions options,
                    std::function<std::chrono::steady_clock::time_point()> now = {});

    /** @brief Accept the "simulation" plugin name and load FrontierNet. */
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

    /** @brief Which detector produced the latest exploration candidates. */
    enum class FrontierSourceKind
    {
        /** @brief No detector produced candidates. */
        kNone,
        /** @brief Occupancy-map frontier detection. */
        kMap,
        /** @brief DeFM depth-model projection. */
        kModel,
    };

    [[nodiscard]] FrontierSourceKind ExplorationFrontierSource() const
    {
        return frontier_source_kind_;
    }

    /** @brief Replace the depth detector (tests only; keeps ONNX out of unit tests). */
    void SetFrontierSourceForTest(std::unique_ptr<FrontierSourceInterface> source);

    [[nodiscard]] std::size_t ExplorationHistorySize() const { return history_.size(); }
    [[nodiscard]] std::string_view ExplorationMode() const { return mode_; }
    [[nodiscard]] const std::optional<common::Pose2D>& ExplorationTarget() const { return target_; }

private:
    /** @brief Frontier waypoints for an exploration goal. */
    std::vector<Waypoint> ExplorationFrontiers(
        const observation::Observation& observation,
        const common::Pose2D& current);

    common::Twist2D Explore(const observation::Observation& observation,
                            const common::Pose2D& current);
    void Remember(const common::Pose2D& pose, bool failed);
    void LogMode(const char* mode);
    struct Visit {
        common::Pose2D pose;
        std::chrono::steady_clock::time_point expires;
        bool failed;
    };
    std::function<std::chrono::steady_clock::time_point()> now_;
    std::deque<Visit> history_;
    std::optional<common::Pose2D> target_;
    config::ExplorationOptions exploration_options_;
    std::chrono::steady_clock::time_point target_started_{}, progress_time_{};
    double best_distance_{0};
    bool scanning_{false}, scan_done_{false};
    double scan_yaw_{0}, scan_rotation_{0};
    std::uint64_t map_hash_{0};
    std::string mode_;
    double clearance_m_{0.4};
    /** @brief True after Configure succeeds. */
    bool configured_{false};
    /** @brief Depth frontier source (null when disabled). */
    std::unique_ptr<FrontierSourceInterface> frontier_source_;
    /** @brief Detector behind the latest candidate list. */
    FrontierSourceKind frontier_source_kind_{FrontierSourceKind::kNone};
    /** @brief Detector behind the active target. */
    FrontierSourceKind target_source_{FrontierSourceKind::kNone};
    /** @brief FrontierNet options from the configuration. */
    config::FrontierNetOptions frontiernet_options_;
    /** @brief Waypoints from the last inference. */
    std::vector<Waypoint> cached_frontiers_;
    /** @brief Pose the cache was computed at. */
    common::Pose2D cached_pose_;
    /** @brief True when cached_frontiers_ is usable. */
    bool cache_valid_{false};
    /** @brief Time of the last inference. */
    std::chrono::steady_clock::time_point last_inference_{};

};

}  // namespace simulation
}  // namespace planning
}  // namespace navigation
