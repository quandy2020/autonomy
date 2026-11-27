/* Copyright 2026 The Openbot Authors. */
/** @file rolling_viewpoint_field.cpp
 * @brief Transactional rolling 3-D viewpoint-clearance implementation.
 */
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <unordered_set>
#include <utility>
#include <vector>

#include "autonomy/task/exploration/viewpoint/rolling_viewpoint_field.hpp"

namespace autonomy::task::exploration {
namespace {

constexpr std::size_t kCancellationStride = 1024;

bool Finite(const Eigen::Vector3d& value) {
    return value.array().isFinite().all();
}

bool Positive(const Eigen::Vector3i& value) {
    return (value.array() > 0).all();
}

std::optional<std::size_t> CellCount(const Eigen::Vector3i& dimensions) {
    if (!Positive(dimensions))
        return std::nullopt;
    std::size_t count = 1;
    for (int axis = 0; axis < 3; ++axis) {
        const auto extent = static_cast<std::size_t>(dimensions[axis]);
        if (count > std::numeric_limits<std::size_t>::max() / extent) {
            return std::nullopt;
        }
        count *= extent;
    }
    return count;
}

int FloorDiv(int value, int divisor) {
    const int quotient = value / divisor;
    const int remainder = value % divisor;
    return remainder < 0 ? quotient - 1 : quotient;
}

std::size_t Index(const Eigen::Vector3i& index,
                  const Eigen::Vector3i& dimensions) {
    return (static_cast<std::size_t>(index.x()) * dimensions.y() + index.y()) *
               dimensions.z() +
           index.z();
}

bool SameLattice(const RollingViewpointFieldView& view,
                 const perception::ObservedVolumeView& volume) {
    return view.frame_id == volume.frame_id &&
           (view.lattice_origin.array() == volume.origin.array()).all() &&
           (view.source_dimensions.array() == volume.dimensions.array())
               .all() &&
           view.resolution == volume.resolution;
}

}  // namespace

struct RollingViewpointFieldView::Storage {
    Eigen::Vector3i dimensions{0, 0, 0};
    std::vector<ViewpointVoxelState> states;
    std::vector<std::uint32_t> occupied_prefix;
    std::vector<std::uint32_t> unknown_prefix;

    std::size_t PrefixIndex(int x, int y, int z) const {
        return (static_cast<std::size_t>(x) * (dimensions.y() + 1) + y) *
                   (dimensions.z() + 1) +
               z;
    }

    std::size_t Count(const std::vector<std::uint32_t>& prefix,
                      const Eigen::Vector3i& minimum,
                      const Eigen::Vector3i& maximum) const {
        const int x0 = minimum.x();
        const int y0 = minimum.y();
        const int z0 = minimum.z();
        const int x1 = maximum.x() + 1;
        const int y1 = maximum.y() + 1;
        const int z1 = maximum.z() + 1;
        const auto at = [&](int x, int y, int z) -> std::int64_t {
            return prefix[PrefixIndex(x, y, z)];
        };
        const std::int64_t count =
            at(x1, y1, z1) - at(x0, y1, z1) - at(x1, y0, z1) - at(x1, y1, z0) +
            at(x0, y0, z1) + at(x0, y1, z0) + at(x1, y0, z0) - at(x0, y0, z0);
        return static_cast<std::size_t>(count);
    }
};

bool RollingViewpointFieldView::Compatible(
    std::uint64_t expected_epoch, std::uint64_t expected_geometry_revision,
    std::string_view expected_frame) const noexcept {
    return map_epoch == expected_epoch &&
           geometry_revision == expected_geometry_revision &&
           frame_id == expected_frame;
}

ViewpointFieldQuery RollingViewpointFieldView::CheckViewpoint(
    const Pose2d& pose, double floor_height, double robot_radius,
    double robot_height, double horizontal_margin, double vertical_margin,
    double clearance_reserve) const noexcept {
    ViewpointFieldQuery result;
    if (!storage || !std::isfinite(pose.x) || !std::isfinite(pose.y) ||
        !std::isfinite(pose.yaw) || !std::isfinite(floor_height) ||
        !std::isfinite(robot_radius) || robot_radius < 0 ||
        !std::isfinite(robot_height) || robot_height <= 0 ||
        !std::isfinite(horizontal_margin) || horizontal_margin < 0 ||
        !std::isfinite(vertical_margin) || vertical_margin < 0 ||
        !std::isfinite(clearance_reserve) || clearance_reserve < 0 ||
        !std::isfinite(resolution) || resolution <= 0) {
        return result;
    }

    const double radius = robot_radius + horizontal_margin + clearance_reserve;
    const auto lower_index = [&](double coordinate, int axis) {
        return static_cast<int>(
            std::floor((coordinate - lattice_origin[axis]) / resolution));
    };
    const auto upper_index = [&](double coordinate, int axis) {
        return static_cast<int>(std::ceil((coordinate - lattice_origin[axis]) /
                                          resolution)) -
               1;
    };

    Eigen::Vector3i minimum{lower_index(pose.x - radius, 0),
                            lower_index(pose.y - radius, 1),
                            lower_index(floor_height, 2) + 1};
    Eigen::Vector3i maximum{
        upper_index(pose.x + radius, 0), upper_index(pose.y + radius, 1),
        upper_index(floor_height + robot_height + vertical_margin, 2)};
    if ((minimum.array() > maximum.array()).any()) {
        result.eligibility = ViewpointFieldEligibility::kEligible;
        return result;
    }

    if ((minimum.array() < window_origin.array()).any() ||
        (maximum.array() >= (window_origin + window_dimensions).array())
            .any() ||
        (minimum.array() < 0).any() ||
        (maximum.array() >= source_dimensions.array()).any()) {
        return result;
    }

    minimum -= window_origin;
    maximum -= window_origin;
    result.occupied_voxels =
        storage->Count(storage->occupied_prefix, minimum, maximum);
    result.unknown_voxels =
        storage->Count(storage->unknown_prefix, minimum, maximum);
    if (result.occupied_voxels > 0) {
        result.eligibility = ViewpointFieldEligibility::kOccupiedClearance;
    } else if (result.unknown_voxels > 0) {
        result.eligibility = ViewpointFieldEligibility::kUnknownClearance;
    } else {
        result.eligibility = ViewpointFieldEligibility::kEligible;
    }
    return result;
}

RollingViewpointField::RollingViewpointField(
    RollingViewpointFieldOptions options)
    : options_(std::move(options)) {
    const auto cells = CellCount(options_.dimensions);
    const bool prefix_overflow =
        (options_.dimensions.array() == std::numeric_limits<int>::max()).any();
    Eigen::Vector3i prefix_dimensions = options_.dimensions;
    if (!prefix_overflow)
        prefix_dimensions.array() += 1;
    const auto prefix_cells = prefix_overflow ? std::optional<std::size_t>{}
                                              : CellCount(prefix_dimensions);
    if (!cells || !prefix_cells || prefix_overflow ||
        !Positive(options_.rollover_step) ||
        (options_.rollover_step.array() > options_.dimensions.array()).any() ||
        options_.maximum_cells == 0 || *cells > options_.maximum_cells ||
        *cells > std::numeric_limits<std::uint32_t>::max() ||
        options_.maximum_input_voxels == 0) {
        throw std::invalid_argument("invalid rolling viewpoint field options");
    }
}

ViewpointFieldUpdateResult RollingViewpointField::Update(
    const perception::ObservedVolumeView& volume,
    const Eigen::Vector3d& focus_position,
    const std::function<bool()>& cancel_checker) {
    ViewpointFieldUpdateResult result;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        result.generation = generation_;
    }
    if (!Finite(volume.origin) || !Finite(focus_position) ||
        !std::isfinite(volume.resolution) || volume.resolution <= 0 ||
        !std::isfinite(volume.source_stamp) || volume.frame_id.empty() ||
        !Positive(volume.dimensions) || !volume.voxels ||
        volume.voxels->size() > options_.maximum_input_voxels ||
        (options_.require_mapping_synchronized &&
         !volume.mapping_synchronized)) {
        return result;
    }
    const auto source_cells = CellCount(volume.dimensions);
    if (!source_cells)
        return result;
    result.measured_voxels = volume.voxels->size();
    if (cancel_checker && cancel_checker()) {
        result.status = ViewpointFieldUpdateStatus::kCanceled;
        return result;
    }

    std::shared_ptr<const RollingViewpointFieldView> previous;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        previous = published_;
    }
    if (previous && (volume.map_epoch < previous->map_epoch ||
                     (volume.map_epoch == previous->map_epoch &&
                      (volume.geometry_revision < previous->geometry_revision ||
                       volume.source_stamp < previous->source_stamp)))) {
        result.status = ViewpointFieldUpdateStatus::kRejectedStale;
        return result;
    }

    const Eigen::Vector3d relative =
        (focus_position - volume.origin) / volume.resolution;
    Eigen::Vector3i focus_index;
    for (int axis = 0; axis < 3; ++axis) {
        if (relative[axis] < std::numeric_limits<int>::min() ||
            relative[axis] > std::numeric_limits<int>::max()) {
            return result;
        }
        focus_index[axis] = static_cast<int>(std::floor(relative[axis]));
    }
    Eigen::Vector3i window_origin;
    for (int axis = 0; axis < 3; ++axis) {
        const int desired = focus_index[axis] - options_.dimensions[axis] / 2;
        const int quantized = FloorDiv(desired, options_.rollover_step[axis]) *
                              options_.rollover_step[axis];
        const int maximum =
            std::max(0, volume.dimensions[axis] - options_.dimensions[axis]);
        window_origin[axis] = std::clamp(quantized, 0, maximum);
    }

    const auto cell_count = *CellCount(options_.dimensions);
    auto storage = std::make_shared<RollingViewpointFieldView::Storage>();
    storage->dimensions = options_.dimensions;
    storage->states.assign(cell_count, ViewpointVoxelState::kUnknown);

    std::unordered_set<std::size_t> unique_indices;
    unique_indices.reserve(volume.voxels->size());
    for (std::size_t i = 0; i < volume.voxels->size(); ++i) {
        if ((i % kCancellationStride) == 0 && cancel_checker &&
            cancel_checker()) {
            result.status = ViewpointFieldUpdateStatus::kCanceled;
            return result;
        }
        const auto& voxel = (*volume.voxels)[i];
        if ((voxel.index.array() < 0).any() ||
            (voxel.index.array() >= volume.dimensions.array()).any() ||
            (voxel.state != perception::ObservedVoxelState::kFree &&
             voxel.state != perception::ObservedVoxelState::kOccupied)) {
            return result;
        }
        const std::size_t source_index = Index(voxel.index, volume.dimensions);
        if (!unique_indices.insert(source_index).second)
            return result;
        const Eigen::Vector3i local = voxel.index - window_origin;
        if ((local.array() < 0).any() ||
            (local.array() >= options_.dimensions.array()).any()) {
            continue;
        }
        storage->states[Index(local, options_.dimensions)] =
            voxel.state == perception::ObservedVoxelState::kOccupied
                ? ViewpointVoxelState::kOccupied
                : ViewpointVoxelState::kFree;
    }

    const Eigen::Vector3i prefix_dimensions = options_.dimensions.array() + 1;
    const auto prefix_count = *CellCount(prefix_dimensions);
    storage->occupied_prefix.assign(prefix_count, 0);
    storage->unknown_prefix.assign(prefix_count, 0);
    for (int x = 1; x <= options_.dimensions.x(); ++x) {
        if ((x % 4) == 0 && cancel_checker && cancel_checker()) {
            result.status = ViewpointFieldUpdateStatus::kCanceled;
            return result;
        }
        for (int y = 1; y <= options_.dimensions.y(); ++y) {
            for (int z = 1; z <= options_.dimensions.z(); ++z) {
                const auto state = storage->states[Index({x - 1, y - 1, z - 1},
                                                         options_.dimensions)];
                const auto update = [&](std::vector<std::uint32_t>* prefix,
                                        bool included) {
                    const auto at = [&](int px, int py,
                                        int pz) -> std::int64_t {
                        return (*prefix)[storage->PrefixIndex(px, py, pz)];
                    };
                    const std::int64_t value =
                        (included ? 1 : 0) + at(x - 1, y, z) + at(x, y - 1, z) +
                        at(x, y, z - 1) - at(x - 1, y - 1, z) -
                        at(x - 1, y, z - 1) - at(x, y - 1, z - 1) +
                        at(x - 1, y - 1, z - 1);
                    (*prefix)[storage->PrefixIndex(x, y, z)] =
                        static_cast<std::uint32_t>(value);
                };
                update(&storage->occupied_prefix,
                       state == ViewpointVoxelState::kOccupied);
                update(&storage->unknown_prefix,
                       state == ViewpointVoxelState::kUnknown);
            }
        }
    }

    auto next = std::make_shared<RollingViewpointFieldView>();
    next->map_epoch = volume.map_epoch;
    next->geometry_revision = volume.geometry_revision;
    next->source_stamp = volume.source_stamp;
    next->frame_id = volume.frame_id;
    next->observation_cut = volume.observation_cut;
    next->lattice_origin = volume.origin;
    next->source_dimensions = volume.dimensions;
    next->window_origin = window_origin;
    next->window_dimensions = options_.dimensions;
    next->resolution = volume.resolution;
    next->storage = std::move(storage);

    const bool same_lattice = previous && SameLattice(*previous, volume);
    result.epoch_reset =
        previous && (previous->map_epoch != volume.map_epoch || !same_lattice);
    result.rolled =
        previous && same_lattice &&
        (previous->window_origin.array() != window_origin.array()).any();
    const bool same_layout = previous && same_lattice && !result.rolled;
    if (same_layout) {
        for (std::size_t i = 0; i < cell_count; ++i) {
            if (previous->storage->states[i] != next->storage->states[i]) {
                ++result.changed_voxels;
            }
        }
    } else if (previous) {
        result.changed_voxels = cell_count;
    }
    const bool geometry_changed =
        !previous || result.epoch_reset || result.rolled ||
        result.changed_voxels > 0 ||
        previous->geometry_revision != volume.geometry_revision;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (geometry_changed)
            ++generation_;
        next->generation = generation_;
        published_ = next;
        result.generation = generation_;
    }
    result.status = geometry_changed ? ViewpointFieldUpdateStatus::kPublished
                                     : ViewpointFieldUpdateStatus::kUnchanged;
    return result;
}

std::shared_ptr<const RollingViewpointFieldView> RollingViewpointField::Read()
    const noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    return published_;
}

void RollingViewpointField::Clear() noexcept {
    std::lock_guard<std::mutex> lock(mutex_);
    published_.reset();
    ++generation_;
}

}  // namespace autonomy::task::exploration
