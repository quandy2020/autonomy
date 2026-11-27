/* Copyright 2026 The Openbot Authors. */

#include "autonomy/task/exploration/routing/route_line_of_sight_view_producer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace autonomy::task::exploration {
namespace {

bool ValidSafety(const autonomy::navigation::SafetyOptions& safety) {
    return std::isfinite(safety.robot_radius) && safety.robot_radius > 0 &&
           std::isfinite(safety.robot_height) && safety.robot_height > 0 &&
           std::isfinite(safety.horizontal_margin) &&
           safety.horizontal_margin >= 0 &&
           std::isfinite(safety.vertical_margin) && safety.vertical_margin >= 0;
}

bool SameLattice(const RollingViewpointFieldView& field,
                 const perception::ObservedVolumeView& volume) {
    return field.source_stamp == volume.source_stamp &&
           field.resolution == volume.resolution &&
           (field.lattice_origin.array() == volume.origin.array()).all() &&
           (field.source_dimensions.array() == volume.dimensions.array()).all();
}

bool Compatible(const perception::MapReadView& map,
                const RollingViewpointFieldView& field) {
    if (!map.grid || !map.observed_volume || map.map_epoch == 0 ||
        map.geometry_revision == 0 || map.frame_id.empty() ||
        !map.mapping_synchronized ||
        !map.observed_volume->mapping_synchronized || field.generation == 0 ||
        !std::isfinite(field.resolution) || field.resolution <= 0 ||
        !field.Compatible(map.map_epoch, map.geometry_revision, map.frame_id)) {
        return false;
    }
    const auto& volume = *map.observed_volume;
    return volume.map_epoch == map.map_epoch &&
           volume.geometry_revision == map.geometry_revision &&
           volume.frame_id == map.frame_id &&
           volume.observation_cut == map.observation_cut &&
           field.observation_cut == map.observation_cut &&
           SameLattice(field, volume);
}

RouteLineOfSightStatus FieldStatus(ViewpointFieldEligibility eligibility) {
    switch (eligibility) {
        case ViewpointFieldEligibility::kEligible:
            return RouteLineOfSightStatus::kVisible;
        case ViewpointFieldEligibility::kEpochMismatch:
        case ViewpointFieldEligibility::kGeometryRevisionMismatch:
        case ViewpointFieldEligibility::kFrameMismatch:
            return RouteLineOfSightStatus::kUnavailable;
        case ViewpointFieldEligibility::kOutsideWindow:
        case ViewpointFieldEligibility::kUnknownClearance:
        case ViewpointFieldEligibility::kOccupiedClearance:
            return RouteLineOfSightStatus::kBlocked;
    }
    return RouteLineOfSightStatus::kUnavailable;
}

}  // namespace

RouteLineOfSightViewProducer::RouteLineOfSightViewProducer(
    RouteLineOfSightViewProducerOptions options)
    : options_(std::move(options)) {
    if (!ValidSafety(options_.safety) ||
        !std::isfinite(options_.clearance_reserve) ||
        options_.clearance_reserve < 0 ||
        options_.maximum_samples_per_check == 0 ||
        options_.maximum_samples_per_check > 1000000) {
        throw std::invalid_argument("route LOS geometry policy is invalid");
    }
}

std::optional<RouteLineOfSightView> RouteLineOfSightViewProducer::Produce(
    const perception::MapReadView& map,
    std::shared_ptr<const RollingViewpointFieldView> field,
    std::function<bool()> canceled) const {
    if (!field || !Compatible(map, *field))
        return std::nullopt;

    auto planar_safety = options_.safety;
    if (map.footprint_inflated) {
        planar_safety.robot_radius = 1e-9;
        planar_safety.horizontal_margin = 0;
    }
    if (map.startup_support)
        planar_safety.startup_support = map.startup_support;

    const auto grid = map.grid;
    const auto body_safety = options_.safety;
    const double clearance_reserve = options_.clearance_reserve;
    const std::size_t maximum_samples = options_.maximum_samples_per_check;
    const double pitch = 0.5 * std::min(grid->resolution(), field->resolution);
    if (!std::isfinite(pitch) || pitch <= 0)
        return std::nullopt;
    std::shared_ptr<const autonomy::navigation::GeometrySafety> geometry;
    try {
        geometry = std::make_shared<const autonomy::navigation::GeometrySafety>(
            grid, planar_safety);
    } catch (const std::invalid_argument&) {
        return std::nullopt;
    }

    RouteLineOfSightView view;
    view.map_epoch = map.map_epoch;
    view.geometry_revision = map.geometry_revision;
    view.field_generation = field->generation;
    view.frame_id = map.frame_id;
    view.check = [grid, field = std::move(field), geometry, body_safety,
                  clearance_reserve, maximum_samples, pitch,
                  canceled = std::move(canceled)](
                     const autonomy::navigation::Pose2d& from,
                     const autonomy::navigation::Pose2d& to) {
        if (!autonomy::navigation::IsFinite(from) ||
            !autonomy::navigation::IsFinite(to)) {
            return RouteLineOfSightStatus::kUnavailable;
        }
        if (canceled && canceled())
            return RouteLineOfSightStatus::kCanceled;

        if (!geometry->VerifySegment(from, to).safe ||
            !geometry->VerifySegment(to, from).safe) {
            return RouteLineOfSightStatus::kBlocked;
        }

        const double distance = autonomy::navigation::Distance(from, to);
        if (!std::isfinite(distance))
            return RouteLineOfSightStatus::kUnavailable;
        const double required_segments = std::ceil(distance / pitch);
        if (!std::isfinite(required_segments) || required_segments < 0 ||
            required_segments >= static_cast<double>(maximum_samples)) {
            return RouteLineOfSightStatus::kUnavailable;
        }
        const auto segments = static_cast<std::size_t>(required_segments);
        const std::size_t samples = segments + 1;
        for (std::size_t index = 0; index < samples; ++index) {
            if (canceled && canceled())
                return RouteLineOfSightStatus::kCanceled;
            const double alpha =
                segments == 0 ? 0 : static_cast<double>(index) / segments;
            autonomy::navigation::Pose2d sample{
                from.x + alpha * (to.x - from.x),
                from.y + alpha * (to.y - from.y), from.yaw};
            const auto cell = grid->WorldToCell(sample);
            if (!cell)
                return RouteLineOfSightStatus::kBlocked;
            const double floor_height = grid->At(cell->x, cell->y).floor_height;
            if (!std::isfinite(floor_height))
                return RouteLineOfSightStatus::kUnavailable;
            const auto checked = field->CheckViewpoint(
                sample, floor_height, body_safety.robot_radius,
                body_safety.robot_height, body_safety.horizontal_margin,
                body_safety.vertical_margin, clearance_reserve + 0.5 * pitch);
            const auto status = FieldStatus(checked.eligibility);
            if (status != RouteLineOfSightStatus::kVisible)
                return status;
        }
        return RouteLineOfSightStatus::kVisible;
    };
    return view;
}

}  // namespace autonomy::task::exploration
