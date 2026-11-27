/* Copyright 2026 The Openbot Authors. */
/** @file rolling_viewpoint_field_test.cpp
 * @brief Focused regressions for persistent rolling 3-D viewpoint clearance.
 */
#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>

#include "autonomy/task/exploration/viewpoint/rolling_viewpoint_field.hpp"

namespace {
namespace e = autonomy::task::exploration;
namespace p = autonomy::perception;

void Require(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}

e::RollingViewpointFieldOptions Options() {
    e::RollingViewpointFieldOptions options;
    options.dimensions = {4, 4, 4};
    options.rollover_step = {2, 2, 2};
    options.maximum_cells = 64;
    options.maximum_input_voxels = 512;
    return options;
}

p::ObservedVolumeView Volume(std::vector<p::ObservedVoxel> voxels,
                             std::uint64_t epoch = 1,
                             std::uint64_t revision = 1, double stamp = 1,
                             Eigen::Vector3i dimensions = {4, 4, 4}) {
    p::ObservedVolumeView volume;
    volume.map_epoch = epoch;
    volume.geometry_revision = revision;
    volume.source_stamp = stamp;
    volume.frame_id = "map";
    volume.dimensions = dimensions;
    volume.resolution = 1;
    volume.mapping_synchronized = true;
    volume.voxels = std::make_shared<const std::vector<p::ObservedVoxel>>(
        std::move(voxels));
    return volume;
}

e::Pose2d Pose() {
    e::Pose2d pose;
    pose.x = 1.5;
    pose.y = 1.5;
    pose.yaw = 0;
    return pose;
}

std::vector<p::ObservedVoxel> FreeColumn() {
    return {{{1, 1, 0}, p::ObservedVoxelState::kOccupied},
            {{1, 1, 1}, p::ObservedVoxelState::kFree},
            {{1, 1, 2}, p::ObservedVoxelState::kFree}};
}

void MeasuredColumnAndOverhang() {
    e::RollingViewpointField field(Options());
    const auto update = field.Update(Volume(FreeColumn()), {1.5, 1.5, 1.5});
    Require(update.status == e::ViewpointFieldUpdateStatus::kPublished,
            "initial measured field was not published");
    auto view = field.Read();
    Require(view && view->Compatible(1, 1, "map"),
            "published field identity is wrong");
    const auto free = view->CheckViewpoint(Pose(), .1, 0, 2, 0, 0);
    Require(free.eligible(), "measured free body column was rejected");

    auto blocked_voxels = FreeColumn();
    blocked_voxels[2].state = p::ObservedVoxelState::kOccupied;
    const auto blocked_update = field.Update(
        Volume(std::move(blocked_voxels), 1, 2, 2), {1.5, 1.5, 1.5});
    Require(blocked_update.status == e::ViewpointFieldUpdateStatus::kPublished,
            "overhang geometry did not advance the field");
    const auto blocked = field.Read()->CheckViewpoint(Pose(), .1, 0, 2, 0, 0);
    Require(
        blocked.eligibility == e::ViewpointFieldEligibility::kOccupiedClearance,
        "occupied overhang did not block the viewpoint");
}

void UnknownNeverBecomesFree() {
    e::RollingViewpointField field(Options());
    auto voxels = FreeColumn();
    voxels.erase(voxels.begin() + 1);
    field.Update(Volume(std::move(voxels)), {1.5, 1.5, 1.5});
    const auto query = field.Read()->CheckViewpoint(Pose(), .1, 0, 2, 0, 0);
    Require(
        query.eligibility == e::ViewpointFieldEligibility::kUnknownClearance,
        "an omitted voxel was interpreted as free");
}

void FullSnapshotOmissionInvalidatesHistory() {
    e::RollingViewpointField field(Options());
    field.Update(Volume(FreeColumn()), {1.5, 1.5, 1.5});
    const auto original = field.Read();
    Require(original->CheckViewpoint(Pose(), .1, 0, 2, 0, 0).eligible(),
            "test precondition is not eligible");

    auto next = FreeColumn();
    next.erase(next.begin() + 2);
    field.Update(Volume(std::move(next), 1, 1, 2), {1.5, 1.5, 1.5});
    Require(field.Read()->CheckViewpoint(Pose(), .1, 0, 2, 0, 0).eligibility ==
                e::ViewpointFieldEligibility::kUnknownClearance,
            "missing full-snapshot evidence retained stale free state");
    Require(original->CheckViewpoint(Pose(), .1, 0, 2, 0, 0).eligible(),
            "publication mutated an outstanding immutable read view");
}

void StaleAndMalformedUpdatesAreTransactional() {
    e::RollingViewpointField field(Options());
    field.Update(Volume(FreeColumn(), 1, 4, 4), {1.5, 1.5, 1.5});
    const auto generation = field.Read()->generation;
    const auto stale = field.Update(Volume({}, 1, 3, 5), {1.5, 1.5, 1.5});
    Require(stale.status == e::ViewpointFieldUpdateStatus::kRejectedStale,
            "regressing geometry revision was accepted");

    auto duplicate = FreeColumn();
    duplicate.push_back(duplicate.back());
    const auto invalid =
        field.Update(Volume(std::move(duplicate), 1, 5, 5), {1.5, 1.5, 1.5});
    Require(invalid.status == e::ViewpointFieldUpdateStatus::kRejectedInvalid,
            "duplicate sparse voxel was accepted");
    Require(field.Read()->generation == generation &&
                field.Read()->CheckViewpoint(Pose(), .1, 0, 2, 0, 0).eligible(),
            "rejected input changed the published field");
}

void RolloverAndOutsideWindow() {
    e::RollingViewpointField field(Options());
    auto first = Volume(FreeColumn(), 1, 1, 1, {8, 4, 4});
    field.Update(first, {1.5, 1.5, 1.5});
    const auto initial_origin = field.Read()->window_origin;
    auto second = Volume(FreeColumn(), 1, 1, 2, {8, 4, 4});
    const auto update = field.Update(second, {5.5, 1.5, 1.5});
    Require(update.rolled &&
                (field.Read()->window_origin.array() != initial_origin.array())
                    .any(),
            "focus movement did not roll the bounded field");
    Require(field.Read()->CheckViewpoint(Pose(), .1, 0, 2, 0, 0).eligibility ==
                e::ViewpointFieldEligibility::kOutsideWindow,
            "viewpoint outside the rolled window was admitted");
}

void CancellationPreservesPublication() {
    e::RollingViewpointField field(Options());
    field.Update(Volume(FreeColumn()), {1.5, 1.5, 1.5});
    const auto before = field.Read();
    const auto canceled =
        field.Update(Volume({}, 1, 2, 2), {1.5, 1.5, 1.5}, [] { return true; });
    Require(canceled.status == e::ViewpointFieldUpdateStatus::kCanceled,
            "canceled update returned the wrong status");
    Require(field.Read() == before,
            "canceled update replaced the published immutable view");
}

}  // namespace

int main() {
    try {
        MeasuredColumnAndOverhang();
        UnknownNeverBecomesFree();
        FullSnapshotOmissionInvalidatesHistory();
        StaleAndMalformedUpdatesAreTransactional();
        RolloverAndOutsideWindow();
        CancellationPreservesPublication();
    } catch (const std::exception& error) {
        std::cerr << "rolling_viewpoint_field_test: " << error.what() << '\n';
        return 1;
    }
    std::cout << "rolling_viewpoint_field_test passed\n";
    return 0;
}
