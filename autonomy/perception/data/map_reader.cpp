/* Copyright 2026 The Openbot Authors. */
#include "autonomy/perception/data/map_reader.hpp"
#include <algorithm>
#include <climits>
#include <stdexcept>

namespace autonomy::perception {
namespace {
/** @brief Unknown numeric evidence is equal to itself for geometric change detection. */
bool EqualHeight(double first, double second) {
    return first == second || (std::isnan(first) && std::isnan(second));
}
bool Equal(const opt::navigation::GridCell& a, const opt::navigation::GridCell& b) {
    return a.state == b.state && EqualHeight(a.floor_height,b.floor_height) &&
           EqualHeight(a.ceiling_height,b.ceiling_height);
}
bool OnLattice(double n) { return std::isfinite(n) && std::abs(n-std::round(n)) < 1e-6 &&
                                 n > INT_MIN/2 && n < INT_MAX/2; }
}  // namespace
MapReader::MapReader(std::size_t maximum_cells) : maximum_cells_(maximum_cells) {
    if (!maximum_cells) throw std::invalid_argument("map history capacity is zero");
}
MapReadView MapReader::Read(const opt::navigation::GridMap& map, double stamp,
    std::uint64_t alignment, std::uint64_t watermark, bool inflated, UnknownCellPolicy unknowns) {
    if (!std::isfinite(stamp) || map.size() > maximum_cells_ || map.frame_id().empty())
        throw std::invalid_argument("invalid map snapshot");
    for (std::size_t i = 0; i < map.size(); ++i) {
        auto index = map.FromIndex(i); const auto& cell = map.At(index.x, index.y);
        if (cell.state != opt::navigation::CellState::kUnknown && cell.state != opt::navigation::CellState::kFree &&
            cell.state != opt::navigation::CellState::kOccupied)
            throw std::invalid_argument("invalid map cell state");
        if ((!std::isfinite(cell.floor_height) && !std::isnan(cell.floor_height)) ||
            (cell.ceiling_height == -std::numeric_limits<double>::infinity()) ||
            (std::isfinite(cell.floor_height) && !std::isnan(cell.ceiling_height) && cell.ceiling_height < cell.floor_height))
            throw std::invalid_argument("invalid map height evidence");
        if (cell.state == opt::navigation::CellState::kFree &&
            (!std::isfinite(cell.floor_height) || std::isnan(cell.ceiling_height) || cell.ceiling_height < cell.floor_height))
            throw std::invalid_argument("invalid map height evidence");
    }
    if (unknowns != UnknownCellPolicy::kRetainHistory && unknowns != UnknownCellPolicy::kReplace)
        throw std::invalid_argument("unsupported unknown-cell map policy");
    const bool reset = !latest_ || alignment != alignment_id_ ||
        resolution_ != map.resolution() || frame_id_ != map.frame_id() ||
        !OnLattice((map.origin_x()-anchor_x_)/map.resolution()) ||
        !OnLattice((map.origin_y()-anchor_y_)/map.resolution()) ||
        latest_->footprint_inflated != inflated;
    if (!reset && (stamp < latest_->source_stamp || watermark < latest_->observation_watermark))
        throw std::invalid_argument("out-of-order map snapshot");
    // Stage changes first so a capacity/validation exception cannot corrupt the reader.
    auto staged = reset ? decltype(history_){} : history_;
    const double ax = reset ? map.origin_x() : anchor_x_;
    const double ay = reset ? map.origin_y() : anchor_y_;
    const int ox = static_cast<int>(std::llround((map.origin_x()-ax)/map.resolution()));
    const int oy = static_cast<int>(std::llround((map.origin_y()-ay)/map.resolution()));
    std::vector<Bounds2d> dirty;
    for (int y = 0; y < map.height(); ++y) for (int x = 0; x < map.width(); ++x) {
        const auto key = std::make_pair(x+ox, y+oy);
        const auto& cell = map.At(x, y);
        auto old = staged.find(key);
        // Unknown in a newly rolled-in window is absence of evidence, not deletion.
        if (unknowns == UnknownCellPolicy::kRetainHistory && cell.state == opt::navigation::CellState::kUnknown &&
            old != staged.end()) continue;
        if (old == staged.end() || !Equal(old->second, cell)) {
            staged[key] = cell;
            const double wx = ax+key.first*map.resolution(), wy = ay+key.second*map.resolution();
            dirty.push_back({wx, wy, wx+map.resolution(), wy+map.resolution()});
        }
    }
    int min_x = ox, min_y = oy, max_x = ox+map.width()-1, max_y = oy+map.height()-1;
    for (const auto& entry : staged) {
        min_x = std::min(min_x, entry.first.first); max_x = std::max(max_x, entry.first.first);
        min_y = std::min(min_y, entry.first.second); max_y = std::max(max_y, entry.first.second);
    }
    const std::uint64_t width = static_cast<std::uint64_t>(max_x-min_x)+1;
    const std::uint64_t height = static_cast<std::uint64_t>(max_y-min_y)+1;
    if (width > maximum_cells_ || height > maximum_cells_ || width*height > maximum_cells_)
        throw std::length_error("map history extent exceeds configured capacity");
    const bool changed = reset || !dirty.empty();
    std::shared_ptr<const opt::navigation::GridMap> merged;
    if (!changed) merged = latest_->grid;
    else {
        auto grid = std::make_shared<opt::navigation::GridMap>(
            static_cast<int>(width), static_cast<int>(height), map.resolution(),
            ax+min_x*map.resolution(), ay+min_y*map.resolution(), map.frame_id());
        for (const auto& entry : staged)
            grid->Set(entry.first.first-min_x, entry.first.second-min_y, entry.second);
        merged = std::move(grid);
    }
    if (reset) { ++epoch_; revision_ = 0; }
    if (changed) ++revision_;
    history_ = std::move(staged); anchor_x_ = ax; anchor_y_ = ay;
    alignment_id_ = alignment; resolution_ = map.resolution(); frame_id_ = map.frame_id();
    latest_ = MapReadView{epoch_, revision_, watermark, stamp, frame_id_, merged,
                          std::move(dirty), inflated, {}, false, true, std::nullopt, {}, 0};
    return *latest_;
}
}  // namespace autonomy::perception
