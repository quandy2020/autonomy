/* Copyright 2026 The Openbot Authors. */
#include "autonomy/planning/planner/vista/native_costmap_reader.hpp"

#include <cmath>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <vector>
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/map/costmap_2d/costmap_2d.hpp"

namespace autonomy::planning::planner::vista {

NativeCostmapReader::NativeCostmapReader(std::size_t maximum_cells)
    : maximum_cells_(maximum_cells), reader_(maximum_cells) {}

perception::MapReadView NativeCostmapReader::Read(
    map::costmap_2d::Costmap2D& costmap, const std::string& frame_id,
    const CostmapReadMetadata& metadata, bool footprint_inflated) {
    std::lock_guard<std::mutex> read_lock(mutex_);
    if (frame_id.empty() || !metadata.alignment_id ||
        !std::isfinite(metadata.observation_source_stamp) ||
        metadata.observation_source_stamp < 0)
        throw std::invalid_argument("invalid native Costmap read metadata");

    unsigned width, height;
    double resolution, origin_x, origin_y;
    std::vector<unsigned char> costs;
    {
        std::unique_lock<map::costmap_2d::Costmap2D::mutex_t> lock(*costmap.getMutex());
        width = costmap.getSizeInCellsX(); height = costmap.getSizeInCellsY();
        resolution = costmap.getResolution();
        origin_x = costmap.getOriginX(); origin_y = costmap.getOriginY();
        if (!width || !height || width > static_cast<unsigned>(std::numeric_limits<int>::max()) ||
            height > static_cast<unsigned>(std::numeric_limits<int>::max()) ||
            static_cast<std::uint64_t>(width)*height > maximum_cells_ ||
            !std::isfinite(resolution) || resolution <= 0 ||
            !std::isfinite(origin_x) || !std::isfinite(origin_y) || !costmap.getCharMap())
            throw std::invalid_argument("invalid native Costmap geometry");
        const std::size_t count = static_cast<std::size_t>(width)*height;
        costs.assign(costmap.getCharMap(), costmap.getCharMap()+count);
    }

    opt::navigation::GridMap snapshot(static_cast<int>(width), static_cast<int>(height),
                                     resolution, origin_x, origin_y, frame_id);
    for (std::size_t i = 0; i < costs.size(); ++i) {
        opt::navigation::GridCell cell;
        const unsigned char blocked = footprint_inflated
            ? map::costmap_2d::INSCRIBED_INFLATED_OBSTACLE
            : map::costmap_2d::LETHAL_OBSTACLE;
        if (costs[i] == map::costmap_2d::NO_INFORMATION)
            cell.state = opt::navigation::CellState::kUnknown;
        else cell.state = costs[i] >= blocked
            ? opt::navigation::CellState::kOccupied : opt::navigation::CellState::kFree;
        const auto index = snapshot.FromIndex(i);
        snapshot.Set(index.x, index.y, cell);
    }
    auto view = reader_.Read(snapshot, metadata.observation_source_stamp,
                            metadata.alignment_id, 0, footprint_inflated);
    view.observation_cut = metadata.observation_cut;
    view.mapping_synchronized = metadata.fusion_confirmed;
    view.has_height_evidence = false;
    return view;
}

}  // namespace autonomy::planning::planner::vista
