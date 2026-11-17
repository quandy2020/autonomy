/* Copyright 2026 The Openbot Authors. */
#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <utility>
#include <vector>
#include "opt/navigation/grid_map.h"
#include "autonomy/perception/data/sensor_observation.hpp"

namespace autonomy::perception {
using Pose2d = opt::navigation::Pose2d;
struct Bounds2d {
    double min_x = 0, min_y = 0, max_x = 0, max_y = 0;
    bool Intersects(const Bounds2d& b) const {
        return min_x <= b.max_x && max_x >= b.min_x &&
               min_y <= b.max_y && max_y >= b.min_y;
    }
    bool Contains(const Pose2d& p) const {
        return p.x >= min_x && p.x <= max_x && p.y >= min_y && p.y <= max_y;
    }
};
struct MapReadView {
    std::uint64_t map_epoch = 0, geometry_revision = 0, observation_watermark = 0;
    double source_stamp = 0;
    std::string frame_id;
    std::shared_ptr<const opt::navigation::GridMap> grid;
    std::vector<Bounds2d> dirty_regions;
    bool footprint_inflated = false;
    perception::ObservationCut observation_cut{};
    // Set only by an integration barrier whose map-fusion semantics are verified.
    bool mapping_synchronized = false;
    // False for a planar Costmap projection: floor/ceiling are route-model
    // values, not measured 3D clearance. Execution must use raw 3D evidence.
    bool has_height_evidence = true;
};
class MapReader {
public:
    explicit MapReader(std::size_t maximum_cells = 2000000);
    // Takes an owned copy obtained through the existing map's locked reader.
    // alignment_id changes on a known map/odom realignment, not on publication.
    MapReadView Read(const opt::navigation::GridMap& snapshot, double source_stamp,
                     std::uint64_t alignment_id, std::uint64_t observation_watermark,
                     bool footprint_inflated = false);
    const std::optional<MapReadView>& latest() const { return latest_; }
private:
    std::size_t maximum_cells_;
    std::map<std::pair<int, int>, opt::navigation::GridCell> history_;
    std::optional<MapReadView> latest_;
    std::uint64_t alignment_id_ = 0, epoch_ = 0, revision_ = 0;
    double resolution_ = 0, anchor_x_ = 0, anchor_y_ = 0;
    std::string frame_id_;
};
}  // namespace autonomy::perception
