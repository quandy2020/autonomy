/* Copyright 2026 The Openbot Authors. */
/**
 * @file route_line_of_sight_view_producer.hpp
 * @brief Immutable planar and measured-3-D route line-of-sight adapter.
 */
#pragma once

#include <cstddef>
#include <functional>
#include <memory>
#include <optional>

#include "autonomy/common/navigation/reachability.hpp"
#include "autonomy/perception/data/map_reader.hpp"
#include "autonomy/task/exploration/routing/route_lookahead_selector.hpp"
#include "autonomy/task/exploration/viewpoint/rolling_viewpoint_field.hpp"

namespace autonomy::task::exploration {

/** @brief Fixed physical-body and bounded-work policy for route LOS checks. */
struct RouteLineOfSightViewProducerOptions {
    autonomy::navigation::SafetyOptions safety;
    double clearance_reserve = 0;
    std::size_t maximum_samples_per_check = 4096;
};

/**
 * @brief Produces provenance-bound LOS views without owning routes or a graph.
 * @details Each produced checker retains only immutable planar and measured 3-D
 * geometry snapshots. Unknown or missing clearance always fails closed.
 */
class RouteLineOfSightViewProducer final
{
public:
    /**
     * @brief Validates and freezes the physical-body and work-limit policy.
     * @throws std::invalid_argument If numeric geometry or the work bound is
     * invalid.
     */
    explicit RouteLineOfSightViewProducer(
        RouteLineOfSightViewProducerOptions options);

    /**
     * @brief Compose one immutable planar and 3-D LOS query view.
     * @param map Planar geometry and synchronized measured-volume provenance.
     * @param field Immutable rolling 3-D clearance view derived from that
     * volume.
     * @param canceled Optional cancellation source, called without map locks.
     * @return No view when evidence is absent, stale, or identity-mismatched.
     */
    std::optional<RouteLineOfSightView> Produce(
        const perception::MapReadView& map,
        std::shared_ptr<const RollingViewpointFieldView> field,
        std::function<bool()> canceled = {}) const;

private:
    RouteLineOfSightViewProducerOptions options_;
};

}  // namespace autonomy::task::exploration
