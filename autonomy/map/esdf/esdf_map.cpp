/*
 * Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file esdf_map.cpp
 * @brief ROS-free FIESTA incremental ESDF (implementation).
 */

#include "autonomy/map/esdf/esdf_map.hpp"

#include <algorithm>
#include <cmath>

#include "autonomy/map/esdf/raycast.hpp"

namespace autonomy {
namespace map {
namespace esdf {
namespace {

// Faces, edges, and two-step faces (FIESTA default 24-neighbourhood).
const Eigen::Vector3i kNeighborOffsets[] = {
    {-1, 0, 0},  {1, 0, 0},  {0, -1, 0}, {0, 1, 0},  {0, 0, -1}, {0, 0, 1},
    {-1, -1, 0}, {1, 1, 0},  {0, -1, -1}, {0, 1, 1}, {-1, 0, -1}, {1, 0, 1},
    {-1, 1, 0},  {1, -1, 0}, {0, -1, 1},  {0, 1, -1}, {1, 0, -1}, {-1, 0, 1},
    {-2, 0, 0},  {2, 0, 0},  {0, -2, 0}, {0, 2, 0},  {0, 0, -2}, {0, 0, 2},
};
constexpr int kNeighborCount =
    static_cast<int>(sizeof(kNeighborOffsets) / sizeof(kNeighborOffsets[0]));

}  // namespace

double EsdfMap::ProbabilityToLogOdds(double probability) {
    return std::log(probability / (1.0 - probability));
}

EsdfMap::EsdfMap(Options options) : options_(std::move(options)) {
    origin_ = options_.origin;
    inverse_resolution_ = 1.0 / options_.resolution;
    for (int axis = 0; axis < 3; ++axis) {
        grid_size_(axis) = static_cast<int>(
            std::ceil(options_.map_size(axis) / options_.resolution));
    }
    world_min_ = origin_;
    world_max_ = origin_ + options_.map_size;
    voxels_per_yz_plane_ = grid_size_(1) * grid_size_(2);
    voxel_count_ = grid_size_(0) * voxels_per_yz_plane_;
    undefined_voxel_index_ = voxel_count_;
    SetOriginalRange();
    SetProbabilities(options_.hit_probability, options_.miss_probability,
                     options_.min_probability, options_.max_probability,
                     options_.occupancy_probability);

    occupancy_buffer_.assign(voxel_count_, 0.0);
    distance_buffer_.assign(voxel_count_, static_cast<double>(kUndefined));
    closest_obstacle_.assign(
        voxel_count_, Eigen::Vector3i(kUndefined, kUndefined, kUndefined));
    hit_count_.assign(voxel_count_, 0);
    observation_count_.assign(voxel_count_, 0);
    obstacle_list_head_.assign(voxel_count_ + 1, kUndefined);
    previous_list_index_.assign(voxel_count_, kUndefined);
    next_list_index_.assign(voxel_count_, kUndefined);
}

void EsdfMap::SetProbabilities(double hit_probability, double miss_probability,
                               double min_probability, double max_probability,
                               double occupancy_probability) {
    hit_log_odds_ = ProbabilityToLogOdds(hit_probability);
    miss_log_odds_ = ProbabilityToLogOdds(miss_probability);
    min_log_odds_ = ProbabilityToLogOdds(min_probability);
    max_log_odds_ = ProbabilityToLogOdds(max_probability);
    occupancy_log_odds_threshold_ = ProbabilityToLogOdds(occupancy_probability);
}

bool EsdfMap::IsOccupied(int index) const {
    return occupancy_buffer_[index] > occupancy_log_odds_threshold_;
}

bool EsdfMap::PositionInMap(const Eigen::Vector3d& position) const {
    return position(0) >= world_min_(0) && position(1) >= world_min_(1) &&
           position(2) >= world_min_(2) && position(0) <= world_max_(0) &&
           position(1) <= world_max_(1) && position(2) <= world_max_(2);
}

bool EsdfMap::VoxelInRange(const Eigen::Vector3i& voxel,
                           bool use_current_range) const {
    const Eigen::Vector3i& lower =
        use_current_range ? update_min_voxel_ : previous_update_min_voxel_;
    const Eigen::Vector3i& upper =
        use_current_range ? update_max_voxel_ : previous_update_max_voxel_;
    return voxel(0) >= lower(0) && voxel(0) <= upper(0) && voxel(1) >= lower(1) &&
           voxel(1) <= upper(1) && voxel(2) >= lower(2) && voxel(2) <= upper(2);
}

void EsdfMap::PositionToVoxel(const Eigen::Vector3d& position,
                              Eigen::Vector3i* voxel) const {
    for (int axis = 0; axis < 3; ++axis) {
        (*voxel)(axis) = static_cast<int>(
            std::floor((position(axis) - origin_(axis)) / options_.resolution));
    }
}

void EsdfMap::VoxelToPosition(const Eigen::Vector3i& voxel,
                              Eigen::Vector3d* position) const {
    for (int axis = 0; axis < 3; ++axis) {
        (*position)(axis) =
            (voxel(axis) + 0.5) * options_.resolution + origin_(axis);
    }
}

int EsdfMap::VoxelToIndex(const Eigen::Vector3i& voxel) const {
    if (voxel(0) == kUndefined) {
        return undefined_voxel_index_;
    }
    return voxel(0) * voxels_per_yz_plane_ + voxel(1) * grid_size_(2) + voxel(2);
}

Eigen::Vector3i EsdfMap::IndexToVoxel(int index) const {
    return {index / voxels_per_yz_plane_,
            (index % voxels_per_yz_plane_) / grid_size_(2),
            index % grid_size_(2)};
}

double EsdfMap::DistanceBetweenVoxels(const Eigen::Vector3i& first,
                                      const Eigen::Vector3i& second) const {
    return (second - first).cast<double>().norm() * options_.resolution;
}

void EsdfMap::DeleteFromList(int list_head_index, int index) {
    if (previous_list_index_[index] != kUndefined) {
        next_list_index_[previous_list_index_[index]] = next_list_index_[index];
    } else {
        obstacle_list_head_[list_head_index] = next_list_index_[index];
    }
    if (next_list_index_[index] != kUndefined) {
        previous_list_index_[next_list_index_[index]] =
            previous_list_index_[index];
    }
    previous_list_index_[index] = next_list_index_[index] = kUndefined;
}

void EsdfMap::InsertIntoList(int list_head_index, int index) {
    if (obstacle_list_head_[list_head_index] == kUndefined) {
        obstacle_list_head_[list_head_index] = index;
    } else {
        previous_list_index_[obstacle_list_head_[list_head_index]] = index;
        next_list_index_[index] = obstacle_list_head_[list_head_index];
        obstacle_list_head_[list_head_index] = index;
    }
}

int EsdfMap::SetOccupancy(const Eigen::Vector3d& position, bool occupied) {
    if (!PositionInMap(position)) {
        return kUndefined;
    }
    Eigen::Vector3i voxel;
    PositionToVoxel(position, &voxel);
    return SetOccupancy(voxel, occupied);
}

int EsdfMap::SetOccupancy(const Eigen::Vector3i& voxel, bool occupied) {
    const int index = VoxelToIndex(voxel);
    if (!VoxelInRange(voxel)) {
        return index;
    }
    observation_count_[index]++;
    hit_count_[index] += occupied ? 1 : 0;
    if (observation_count_[index] == 1) {
        occupancy_queue_.push(QueueElement{voxel, 0.0});
    }
    return index;
}

int EsdfMap::GetOccupancy(const Eigen::Vector3d& position) const {
    if (!PositionInMap(position)) {
        return kUndefined;
    }
    Eigen::Vector3i voxel;
    PositionToVoxel(position, &voxel);
    return GetOccupancy(voxel);
}

int EsdfMap::GetOccupancy(const Eigen::Vector3i& voxel) const {
    return IsOccupied(VoxelToIndex(voxel)) ? 1 : 0;
}

double EsdfMap::GetDistance(const Eigen::Vector3d& position) const {
    if (!PositionInMap(position)) {
        return static_cast<double>(kUndefined);
    }
    Eigen::Vector3i voxel;
    PositionToVoxel(position, &voxel);
    return GetDistance(voxel);
}

double EsdfMap::GetDistance(const Eigen::Vector3i& voxel) const {
    const double distance = distance_buffer_[VoxelToIndex(voxel)];
    return distance < 0.0 ? static_cast<double>(kInfinity) : distance;
}

double EsdfMap::StoredDistance(int index) const {
    const double distance = distance_buffer_[index];
    return distance < 0.0 ? static_cast<double>(kInfinity) : distance;
}

double EsdfMap::GetDistanceWithGradient(const Eigen::Vector3d& position,
                                        Eigen::Vector3d* gradient) const {
    if (!PositionInMap(position) || gradient == nullptr) {
        return -1.0;
    }
    const Eigen::Vector3d half_voxel_shifted_position =
        position - 0.5 * options_.resolution * Eigen::Vector3d::Ones();
    Eigen::Vector3i base_voxel;
    PositionToVoxel(half_voxel_shifted_position, &base_voxel);
    Eigen::Vector3d base_voxel_centre;
    VoxelToPosition(base_voxel, &base_voxel_centre);
    const Eigen::Vector3d interpolation_weight =
        (position - base_voxel_centre) * inverse_resolution_;

    double corner_distance[2][2][2];
    for (int offset_x = 0; offset_x < 2; ++offset_x) {
        for (int offset_y = 0; offset_y < 2; ++offset_y) {
            for (int offset_z = 0; offset_z < 2; ++offset_z) {
                corner_distance[offset_x][offset_y][offset_z] = GetDistance(
                    Eigen::Vector3i(base_voxel +
                                    Eigen::Vector3i(offset_x, offset_y, offset_z)));
            }
        }
    }
    const double interpolant_y0_z0 =
        (1 - interpolation_weight[0]) * corner_distance[0][0][0] +
        interpolation_weight[0] * corner_distance[1][0][0];
    const double interpolant_y0_z1 =
        (1 - interpolation_weight[0]) * corner_distance[0][0][1] +
        interpolation_weight[0] * corner_distance[1][0][1];
    const double interpolant_y1_z0 =
        (1 - interpolation_weight[0]) * corner_distance[0][1][0] +
        interpolation_weight[0] * corner_distance[1][1][0];
    const double interpolant_y1_z1 =
        (1 - interpolation_weight[0]) * corner_distance[0][1][1] +
        interpolation_weight[0] * corner_distance[1][1][1];
    const double interpolant_z0 =
        (1 - interpolation_weight[1]) * interpolant_y0_z0 +
        interpolation_weight[1] * interpolant_y1_z0;
    const double interpolant_z1 =
        (1 - interpolation_weight[1]) * interpolant_y0_z1 +
        interpolation_weight[1] * interpolant_y1_z1;
    const double interpolated_distance =
        (1 - interpolation_weight[2]) * interpolant_z0 +
        interpolation_weight[2] * interpolant_z1;

    (*gradient)[2] = (interpolant_z1 - interpolant_z0) * inverse_resolution_;
    (*gradient)[1] = ((1 - interpolation_weight[2]) *
                          (interpolant_y1_z0 - interpolant_y0_z0) +
                      interpolation_weight[2] *
                          (interpolant_y1_z1 - interpolant_y0_z1)) *
                     inverse_resolution_;
    (*gradient)[0] =
        (1 - interpolation_weight[2]) * (1 - interpolation_weight[1]) *
            (corner_distance[1][0][0] - corner_distance[0][0][0]) +
        (1 - interpolation_weight[2]) * interpolation_weight[1] *
            (corner_distance[1][1][0] - corner_distance[0][1][0]) +
        interpolation_weight[2] * (1 - interpolation_weight[1]) *
            (corner_distance[1][0][1] - corner_distance[0][0][1]) +
        interpolation_weight[2] * interpolation_weight[1] *
            (corner_distance[1][1][1] - corner_distance[0][1][1]);
    (*gradient)[0] *= inverse_resolution_;
    return interpolated_distance;
}

bool EsdfMap::CheckUpdate() const { return !occupancy_queue_.empty(); }

bool EsdfMap::UpdateOccupancy(bool reset_to_full_map) {
    while (!occupancy_queue_.empty()) {
        const QueueElement element = occupancy_queue_.front();
        occupancy_queue_.pop();
        const int index = VoxelToIndex(element.voxel);
        const int previously_occupied = IsOccupied(index);
        const double log_odds_update =
            (hit_count_[index] >= observation_count_[index] - hit_count_[index]
                 ? hit_log_odds_
                 : miss_log_odds_);
        hit_count_[index] = observation_count_[index] = 0;
        if (distance_buffer_[index] < 0.0) {
            distance_buffer_[index] = static_cast<double>(kInfinity);
            InsertIntoList(undefined_voxel_index_, index);
        }
        if ((log_odds_update >= 0.0 && occupancy_buffer_[index] >= max_log_odds_) ||
            (log_odds_update <= 0.0 && occupancy_buffer_[index] <= min_log_odds_)) {
            continue;
        }
        if (!reset_to_full_map && !VoxelInRange(element.voxel, false)) {
            occupancy_buffer_[index] = 0.0;
            distance_buffer_[index] = static_cast<double>(kInfinity);
        }
        occupancy_buffer_[index] = std::min(
            std::max(occupancy_buffer_[index] + log_odds_update, min_log_odds_),
            max_log_odds_);
        if (IsOccupied(index) && !previously_occupied) {
            insert_queue_.push(QueueElement{element.voxel, 0.0});
        } else if (!IsOccupied(index) && previously_occupied) {
            delete_queue_.push(
                QueueElement{element.voxel, static_cast<double>(kInfinity)});
        }
    }
    return !insert_queue_.empty() || !delete_queue_.empty();
}

void EsdfMap::UpdateESDF() {
    while (!insert_queue_.empty()) {
        const QueueElement element = insert_queue_.front();
        insert_queue_.pop();
        const int index = VoxelToIndex(element.voxel);
        if (!IsOccupied(index)) {
            continue;
        }
        DeleteFromList(VoxelToIndex(closest_obstacle_[index]), index);
        closest_obstacle_[index] = element.voxel;
        distance_buffer_[index] = 0.0;
        InsertIntoList(index, index);
        update_queue_.push(element);
    }
    while (!delete_queue_.empty()) {
        const QueueElement element = delete_queue_.front();
        delete_queue_.pop();
        const int index = VoxelToIndex(element.voxel);
        if (IsOccupied(index)) {
            continue;
        }
        int next_occupied_index = kUndefined;
        for (int occupied_index = obstacle_list_head_[index];
             occupied_index != kUndefined; occupied_index = next_occupied_index) {
            closest_obstacle_[occupied_index] =
                Eigen::Vector3i(kUndefined, kUndefined, kUndefined);
            const Eigen::Vector3i occupied_voxel = IndexToVoxel(occupied_index);
            double distance = static_cast<double>(kInfinity);
            for (int neighbor = 0; neighbor < kNeighborCount; ++neighbor) {
                const Eigen::Vector3i neighbor_voxel =
                    occupied_voxel + kNeighborOffsets[neighbor];
                const int neighbor_index = VoxelToIndex(neighbor_voxel);
                if (VoxelInRange(neighbor_voxel) &&
                    closest_obstacle_[neighbor_index](0) != kUndefined &&
                    IsOccupied(VoxelToIndex(closest_obstacle_[neighbor_index]))) {
                    const double candidate = DistanceBetweenVoxels(
                        occupied_voxel, closest_obstacle_[neighbor_index]);
                    if (candidate < distance) {
                        distance = candidate;
                        closest_obstacle_[occupied_index] =
                            closest_obstacle_[neighbor_index];
                    }
                    break;
                }
            }
            previous_list_index_[occupied_index] = kUndefined;
            next_occupied_index = next_list_index_[occupied_index];
            next_list_index_[occupied_index] = kUndefined;
            distance_buffer_[occupied_index] = distance;
            if (distance < static_cast<double>(kInfinity)) {
                update_queue_.push(QueueElement{occupied_voxel, distance});
            }
            InsertIntoList(VoxelToIndex(closest_obstacle_[occupied_index]),
                           occupied_index);
        }
        obstacle_list_head_[index] = kUndefined;
    }

    while (!update_queue_.empty()) {
        const QueueElement element = update_queue_.front();
        update_queue_.pop();
        const int index = VoxelToIndex(element.voxel);
        if (element.distance != distance_buffer_[index]) {
            continue;
        }
        bool distance_changed = false;
        for (int neighbor = 0; neighbor < kNeighborCount; ++neighbor) {
            const Eigen::Vector3i neighbor_voxel =
                element.voxel + kNeighborOffsets[neighbor];
            if (!VoxelInRange(neighbor_voxel)) {
                continue;
            }
            const int neighbor_index = VoxelToIndex(neighbor_voxel);
            if (closest_obstacle_[neighbor_index](0) == kUndefined) {
                continue;
            }
            const double candidate = DistanceBetweenVoxels(
                element.voxel, closest_obstacle_[neighbor_index]);
            if (StoredDistance(index) > candidate) {
                distance_buffer_[index] = candidate;
                distance_changed = true;
                DeleteFromList(VoxelToIndex(closest_obstacle_[index]), index);
                InsertIntoList(VoxelToIndex(closest_obstacle_[neighbor_index]),
                               index);
                closest_obstacle_[index] = closest_obstacle_[neighbor_index];
            }
        }
        if (distance_changed) {
            update_queue_.push(QueueElement{element.voxel, distance_buffer_[index]});
            continue;
        }
        const int closest_obstacle_index = VoxelToIndex(closest_obstacle_[index]);
        for (int neighbor = 0; neighbor < kNeighborCount; ++neighbor) {
            const Eigen::Vector3i neighbor_voxel =
                element.voxel + kNeighborOffsets[neighbor];
            if (!VoxelInRange(neighbor_voxel)) {
                continue;
            }
            const int neighbor_index = VoxelToIndex(neighbor_voxel);
            const double candidate =
                DistanceBetweenVoxels(neighbor_voxel, closest_obstacle_[index]);
            if (StoredDistance(neighbor_index) > candidate) {
                distance_buffer_[neighbor_index] = candidate;
                DeleteFromList(VoxelToIndex(closest_obstacle_[neighbor_index]),
                               neighbor_index);
                InsertIntoList(closest_obstacle_index, neighbor_index);
                closest_obstacle_[neighbor_index] = closest_obstacle_[index];
                update_queue_.push(QueueElement{neighbor_voxel, candidate});
            }
        }
    }
}

void EsdfMap::SetUpdateRange(const Eigen::Vector3d& min_position,
                             const Eigen::Vector3d& max_position,
                             bool remember_previous_range) {
    Eigen::Vector3d clamped_min = min_position.cwiseMax(world_min_);
    Eigen::Vector3d clamped_max = max_position.cwiseMin(world_max_);
    if (remember_previous_range) {
        previous_update_min_voxel_ = update_min_voxel_;
        previous_update_max_voxel_ = update_max_voxel_;
    }
    PositionToVoxel(clamped_min, &update_min_voxel_);
    PositionToVoxel(clamped_max - Eigen::Vector3d::Constant(options_.resolution / 2.0),
                    &update_max_voxel_);
}

void EsdfMap::SetOriginalRange() {
    update_min_voxel_ << 0, 0, 0;
    update_max_voxel_ << grid_size_(0) - 1, grid_size_(1) - 1, grid_size_(2) - 1;
    previous_update_min_voxel_ = update_min_voxel_;
    previous_update_max_voxel_ = update_max_voxel_;
}

void EsdfMap::IntegrateRay(const Eigen::Vector3d& origin,
                           const Eigen::Vector3d& point_in) {
    Eigen::Vector3d point = point_in;
    const double length = (point - origin).norm();
    if (length < options_.min_ray_length) {
        return;
    }
    if (length > options_.max_ray_length) {
        point = (point - origin) / length * options_.max_ray_length + origin;
        SetOccupancy(point, false);
    } else {
        SetOccupancy(point, true);
    }

    std::vector<Eigen::Vector3d> voxels;
    const double resolution = options_.resolution;
    Raycast(origin / resolution, point / resolution, world_min_ / resolution,
            world_max_ / resolution, &voxels);
    const Eigen::Vector3d half_voxel(0.5, 0.5, 0.5);
    for (int step = static_cast<int>(voxels.size()) - 2; step >= 0; --step) {
        const Eigen::Vector3d world =
            (voxels[static_cast<std::size_t>(step)] + half_voxel) * resolution;
        const double along_ray = (world - origin).norm();
        if (along_ray < options_.min_ray_length) {
            break;
        }
        if (along_ray > options_.max_ray_length) {
            continue;
        }
        SetOccupancy(world, false);
    }
}

void EsdfMap::IntegrateCloud(const Eigen::Vector3d& origin,
                             const std::vector<Eigen::Vector3d>& points) {
    for (const auto& point : points) {
        if (!std::isfinite(point.x()) || !std::isfinite(point.y()) ||
            !std::isfinite(point.z())) {
            continue;
        }
        IntegrateRay(origin, point);
    }
}

std::vector<Eigen::Vector3d> EsdfMap::OccupiedPoints() const {
    std::vector<Eigen::Vector3d> points;
    for (int voxel_x = update_min_voxel_(0); voxel_x <= update_max_voxel_(0);
         ++voxel_x) {
        for (int voxel_y = update_min_voxel_(1); voxel_y <= update_max_voxel_(1);
             ++voxel_y) {
            for (int voxel_z = update_min_voxel_(2); voxel_z <= update_max_voxel_(2);
                 ++voxel_z) {
                const Eigen::Vector3i voxel(voxel_x, voxel_y, voxel_z);
                if (!IsOccupied(VoxelToIndex(voxel))) {
                    continue;
                }
                Eigen::Vector3d position;
                VoxelToPosition(voxel, &position);
                points.push_back(position);
            }
        }
    }
    return points;
}

bool EsdfMap::CheckConsistency() const {
    for (int voxel_x = 0; voxel_x < grid_size_(0); ++voxel_x) {
        for (int voxel_y = 0; voxel_y < grid_size_(1); ++voxel_y) {
            for (int voxel_z = 0; voxel_z < grid_size_(2); ++voxel_z) {
                const int index =
                    VoxelToIndex(Eigen::Vector3i(voxel_x, voxel_y, voxel_z));
                if ((previous_list_index_[index] != kUndefined &&
                     next_list_index_[previous_list_index_[index]] != index) ||
                    (next_list_index_[index] != kUndefined &&
                     previous_list_index_[next_list_index_[index]] != index)) {
                    return false;
                }
                if (previous_list_index_[index] == kUndefined &&
                    distance_buffer_[index] >= 0.0 &&
                    obstacle_list_head_[VoxelToIndex(closest_obstacle_[index])] !=
                        index) {
                    return false;
                }
            }
        }
    }
    return true;
}

}  // namespace esdf
}  // namespace map
}  // namespace autonomy
