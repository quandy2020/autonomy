/* Copyright 2026 The Openbot Authors. */
/**
 * @file route_line_of_sight_view_producer_test.cpp
 * @brief Regressions for immutable planar and measured-3-D route LOS views.
 */
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>

#include "autonomy/task/exploration/routing/route_line_of_sight_view_producer.hpp"

namespace {
namespace e = autonomy::task::exploration;
namespace n = autonomy::navigation;
namespace p = autonomy::perception;

void Require(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}

struct Fixture {
    std::shared_ptr<n::GridMap> grid = std::make_shared<n::GridMap>(6, 3, 1.0);
    std::shared_ptr<p::ObservedVolumeView> volume =
        std::make_shared<p::ObservedVolumeView>();
    std::unique_ptr<e::RollingViewpointField> rolling;
    p::MapReadView map;

    Fixture() {
        grid->Fill({n::CellState::kFree, 0});
        volume->map_epoch = 1;
        volume->geometry_revision = 1;
        volume->source_stamp = 10;
        volume->frame_id = "map";
        volume->dimensions = {6, 3, 3};
        volume->resolution = 1;
        volume->mapping_synchronized = true;
        volume->voxels = Voxels(-1);

        map.map_epoch = 1;
        map.geometry_revision = 1;
        map.frame_id = "map";
        map.grid = grid;
        map.mapping_synchronized = true;
        map.observed_volume = volume;

        e::RollingViewpointFieldOptions options;
        options.dimensions = {6, 3, 3};
        options.rollover_step = {6, 3, 3};
        options.maximum_cells = 54;
        options.maximum_input_voxels = 54;
        rolling = std::make_unique<e::RollingViewpointField>(options);
        Publish();
    }

    std::shared_ptr<const std::vector<p::ObservedVoxel>> Voxels(
        int occupied_x) const {
        std::vector<p::ObservedVoxel> voxels;
        for (int y = 0; y < 3; ++y) {
            for (int x = 0; x < 6; ++x) {
                voxels.push_back({{x, y, 0}, p::ObservedVoxelState::kOccupied});
                voxels.push_back({{x, y, 1},
                                  x == occupied_x
                                      ? p::ObservedVoxelState::kOccupied
                                      : p::ObservedVoxelState::kFree});
            }
        }
        return std::make_shared<const std::vector<p::ObservedVoxel>>(
            std::move(voxels));
    }

    void Publish() {
        const auto result = rolling->Update(*volume, {3, 1.5, 1.5});
        Require(result.status == e::ViewpointFieldUpdateStatus::kPublished,
                "fixture field was not published");
    }
};

e::RouteLineOfSightViewProducer Producer(std::size_t maximum_samples = 64) {
    e::RouteLineOfSightViewProducerOptions options;
    options.safety.robot_radius = 0.01;
    options.safety.horizontal_margin = 0;
    options.safety.robot_height = 1.9;
    options.safety.vertical_margin = 0;
    options.maximum_samples_per_check = maximum_samples;
    return e::RouteLineOfSightViewProducer(options);
}

void CompatibleMeasuredSegmentIsVisible() {
    Fixture fixture;
    const auto view = Producer().Produce(fixture.map, fixture.rolling->Read());
    Require(view && view->field_generation != 0,
            "compatible evidence did not produce an LOS view");
    Require(view->check({1.5, 1.5, 0}, {4.5, 1.5, 0}) ==
                e::RouteLineOfSightStatus::kVisible,
            "measured-free direct segment was rejected");
    Require(view->check({1.5, 1.5, 0}, {1.5, 1.5, 0}) ==
                e::RouteLineOfSightStatus::kVisible,
            "zero-length measured pose was rejected");
}

void PlanarAndVolumeObstaclesFailClosed() {
    Fixture planar;
    planar.grid->Set(3, 1, {n::CellState::kOccupied, 0});
    const auto planar_view =
        Producer().Produce(planar.map, planar.rolling->Read());
    Require(planar_view && planar_view->check({1.5, 1.5, 0}, {4.5, 1.5, 0}) ==
                               e::RouteLineOfSightStatus::kBlocked,
            "planar obstacle did not block direct LOS");

    Fixture volume;
    volume.volume->source_stamp = 11;
    volume.volume->geometry_revision = 2;
    volume.volume->voxels = volume.Voxels(3);
    volume.map.geometry_revision = 2;
    volume.Publish();
    const auto volume_view =
        Producer().Produce(volume.map, volume.rolling->Read());
    Require(volume_view && volume_view->check({1.5, 1.5, 0}, {4.5, 1.5, 0}) ==
                               e::RouteLineOfSightStatus::kBlocked,
            "occupied overhang did not block direct LOS");
}

void ProvenanceCancellationAndWorkAreBounded() {
    Fixture fixture;
    auto stale = fixture.map;
    stale.observation_cut.present = 1;
    Require(!Producer().Produce(stale, fixture.rolling->Read()),
            "mismatched fusion cut produced an LOS view");

    const auto canceled = Producer().Produce(
        fixture.map, fixture.rolling->Read(), [] { return true; });
    Require(canceled && canceled->check({1.5, 1.5, 0}, {4.5, 1.5, 0}) ==
                            e::RouteLineOfSightStatus::kCanceled,
            "cancellation was not propagated");

    const auto bounded =
        Producer(2).Produce(fixture.map, fixture.rolling->Read());
    Require(bounded && bounded->check({1.5, 1.5, 0}, {4.5, 1.5, 0}) ==
                           e::RouteLineOfSightStatus::kUnavailable,
            "sample work ceiling did not fail closed");
    Require(bounded->check({std::numeric_limits<double>::quiet_NaN(), 1.5, 0},
                           {4.5, 1.5, 0}) ==
                e::RouteLineOfSightStatus::kUnavailable,
            "non-finite endpoint did not fail closed");
}

}  // namespace

int main() {
    try {
        CompatibleMeasuredSegmentIsVisible();
        PlanarAndVolumeObstaclesFailClosed();
        ProvenanceCancellationAndWorkAreBounded();
    } catch (const std::exception& error) {
        std::cerr << "route_line_of_sight_view_producer_test: " << error.what()
                  << '\n';
        return 1;
    }
    std::cout << "route_line_of_sight_view_producer_test passed\n";
    return 0;
}
