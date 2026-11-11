/*
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
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
const Eigen::Vector3i kDirs[] = {
    {-1, 0, 0},  {1, 0, 0},  {0, -1, 0}, {0, 1, 0},  {0, 0, -1}, {0, 0, 1},
    {-1, -1, 0}, {1, 1, 0},  {0, -1, -1}, {0, 1, 1}, {-1, 0, -1}, {1, 0, 1},
    {-1, 1, 0},  {1, -1, 0}, {0, -1, 1},  {0, 1, -1}, {1, 0, -1}, {-1, 0, 1},
    {-2, 0, 0},  {2, 0, 0},  {0, -2, 0}, {0, 2, 0},  {0, 0, -2}, {0, 0, 2},
};
constexpr int kNumDirs = static_cast<int>(sizeof(kDirs) / sizeof(kDirs[0]));

}  // namespace

double EsdfMap::Logit(double x) { return std::log(x / (1.0 - x)); }

EsdfMap::EsdfMap(Options options) : options_(std::move(options)) {
    origin_ = options_.origin;
    resolution_inv_ = 1.0 / options_.resolution;
    for (int i = 0; i < 3; ++i) {
        grid_size_(i) =
            static_cast<int>(std::ceil(options_.map_size(i) / options_.resolution));
    }
    min_range_ = origin_;
    max_range_ = origin_ + options_.map_size;
    grid_size_yz_ = grid_size_(1) * grid_size_(2);
    grid_total_size_ = grid_size_(0) * grid_size_yz_;
    reserved_idx_4_undefined_ = grid_total_size_;
    SetOriginalRange();
    SetProbabilities(options_.p_hit, options_.p_miss, options_.p_min,
                     options_.p_max, options_.p_occ);

    occupancy_buffer_.assign(grid_total_size_, 0.0);
    distance_buffer_.assign(grid_total_size_, static_cast<double>(kUndefined));
    closest_obstacle_.assign(grid_total_size_,
                             Eigen::Vector3i(kUndefined, kUndefined, kUndefined));
    num_hit_.assign(grid_total_size_, 0);
    num_miss_.assign(grid_total_size_, 0);
    head_.assign(grid_total_size_ + 1, kUndefined);
    prev_.assign(grid_total_size_, kUndefined);
    next_.assign(grid_total_size_, kUndefined);
}

void EsdfMap::SetProbabilities(double p_hit, double p_miss, double p_min,
                               double p_max, double p_occ) {
    prob_hit_log_ = Logit(p_hit);
    prob_miss_log_ = Logit(p_miss);
    clamp_min_log_ = Logit(p_min);
    clamp_max_log_ = Logit(p_max);
    min_occupancy_log_ = Logit(p_occ);
}

bool EsdfMap::Exist(int idx) const {
    return occupancy_buffer_[idx] > min_occupancy_log_;
}

bool EsdfMap::PosInMap(const Eigen::Vector3d& pos) const {
    return pos(0) >= min_range_(0) && pos(1) >= min_range_(1) &&
           pos(2) >= min_range_(2) && pos(0) <= max_range_(0) &&
           pos(1) <= max_range_(1) && pos(2) <= max_range_(2);
}

bool EsdfMap::VoxInRange(const Eigen::Vector3i& vox, bool current_vec) const {
    const Eigen::Vector3i& lo = current_vec ? min_vec_ : last_min_vec_;
    const Eigen::Vector3i& hi = current_vec ? max_vec_ : last_max_vec_;
    return vox(0) >= lo(0) && vox(0) <= hi(0) && vox(1) >= lo(1) &&
           vox(1) <= hi(1) && vox(2) >= lo(2) && vox(2) <= hi(2);
}

void EsdfMap::Pos2Vox(const Eigen::Vector3d& pos, Eigen::Vector3i* vox) const {
    for (int i = 0; i < 3; ++i) {
        (*vox)(i) = static_cast<int>(
            std::floor((pos(i) - origin_(i)) / options_.resolution));
    }
}

void EsdfMap::Vox2Pos(const Eigen::Vector3i& vox, Eigen::Vector3d* pos) const {
    for (int i = 0; i < 3; ++i) {
        (*pos)(i) = (vox(i) + 0.5) * options_.resolution + origin_(i);
    }
}

int EsdfMap::Vox2Idx(const Eigen::Vector3i& vox) const {
    if (vox(0) == kUndefined) {
        return reserved_idx_4_undefined_;
    }
    return vox(0) * grid_size_yz_ + vox(1) * grid_size_(2) + vox(2);
}

Eigen::Vector3i EsdfMap::Idx2Vox(int idx) const {
    return {idx / grid_size_yz_, (idx % grid_size_yz_) / grid_size_(2),
            idx % grid_size_(2)};
}

double EsdfMap::Dist(const Eigen::Vector3i& a, const Eigen::Vector3i& b) const {
    return (b - a).cast<double>().norm() * options_.resolution;
}

void EsdfMap::DeleteFromList(int link, int idx) {
    if (prev_[idx] != kUndefined) {
        next_[prev_[idx]] = next_[idx];
    } else {
        head_[link] = next_[idx];
    }
    if (next_[idx] != kUndefined) {
        prev_[next_[idx]] = prev_[idx];
    }
    prev_[idx] = next_[idx] = kUndefined;
}

void EsdfMap::InsertIntoList(int link, int idx) {
    if (head_[link] == kUndefined) {
        head_[link] = idx;
    } else {
        prev_[head_[link]] = idx;
        next_[idx] = head_[link];
        head_[link] = idx;
    }
}

int EsdfMap::SetOccupancy(const Eigen::Vector3d& pos, bool occupied) {
    if (!PosInMap(pos)) {
        return kUndefined;
    }
    Eigen::Vector3i vox;
    Pos2Vox(pos, &vox);
    return SetOccupancy(vox, occupied);
}

int EsdfMap::SetOccupancy(const Eigen::Vector3i& vox, bool occupied) {
    const int idx = Vox2Idx(vox);
    if (!VoxInRange(vox)) {
        return idx;
    }
    num_miss_[idx]++;
    num_hit_[idx] += occupied ? 1 : 0;
    if (num_miss_[idx] == 1) {
        occupancy_queue_.push(QueueElement{vox, 0.0});
    }
    return idx;
}

int EsdfMap::GetOccupancy(const Eigen::Vector3d& pos) const {
    if (!PosInMap(pos)) {
        return kUndefined;
    }
    Eigen::Vector3i vox;
    Pos2Vox(pos, &vox);
    return GetOccupancy(vox);
}

int EsdfMap::GetOccupancy(const Eigen::Vector3i& vox) const {
    return Exist(Vox2Idx(vox)) ? 1 : 0;
}

double EsdfMap::GetDistance(const Eigen::Vector3d& pos) const {
    if (!PosInMap(pos)) {
        return static_cast<double>(kUndefined);
    }
    Eigen::Vector3i vox;
    Pos2Vox(pos, &vox);
    return GetDistance(vox);
}

double EsdfMap::GetDistance(const Eigen::Vector3i& vox) const {
    const double d = distance_buffer_[Vox2Idx(vox)];
    return d < 0.0 ? static_cast<double>(kInfinity) : d;
}

double EsdfMap::StoredDistance(int idx) const {
    const double d = distance_buffer_[idx];
    return d < 0.0 ? static_cast<double>(kInfinity) : d;
}

double EsdfMap::GetDistanceWithGradient(const Eigen::Vector3d& pos,
                                        Eigen::Vector3d* grad) const {
    if (!PosInMap(pos) || grad == nullptr) {
        return -1.0;
    }
    const Eigen::Vector3d pos_m =
        pos - 0.5 * options_.resolution * Eigen::Vector3d::Ones();
    Eigen::Vector3i idx;
    Pos2Vox(pos_m, &idx);
    Eigen::Vector3d idx_pos;
    Vox2Pos(idx, &idx_pos);
    const Eigen::Vector3d diff = (pos - idx_pos) * resolution_inv_;

    double values[2][2][2];
    for (int x = 0; x < 2; ++x) {
        for (int y = 0; y < 2; ++y) {
            for (int z = 0; z < 2; ++z) {
                values[x][y][z] =
                    GetDistance(Eigen::Vector3i(idx + Eigen::Vector3i(x, y, z)));
            }
        }
    }
    const double v00 =
        (1 - diff[0]) * values[0][0][0] + diff[0] * values[1][0][0];
    const double v01 =
        (1 - diff[0]) * values[0][0][1] + diff[0] * values[1][0][1];
    const double v10 =
        (1 - diff[0]) * values[0][1][0] + diff[0] * values[1][1][0];
    const double v11 =
        (1 - diff[0]) * values[0][1][1] + diff[0] * values[1][1][1];
    const double v0 = (1 - diff[1]) * v00 + diff[1] * v10;
    const double v1 = (1 - diff[1]) * v01 + diff[1] * v11;
    const double dist = (1 - diff[2]) * v0 + diff[2] * v1;

    (*grad)[2] = (v1 - v0) * resolution_inv_;
    (*grad)[1] =
        ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) * resolution_inv_;
    (*grad)[0] = (1 - diff[2]) * (1 - diff[1]) *
                     (values[1][0][0] - values[0][0][0]) +
                 (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]) +
                 diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]) +
                 diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);
    (*grad)[0] *= resolution_inv_;
    return dist;
}

bool EsdfMap::CheckUpdate() const { return !occupancy_queue_.empty(); }

bool EsdfMap::UpdateOccupancy(bool global_map) {
    while (!occupancy_queue_.empty()) {
        const QueueElement xx = occupancy_queue_.front();
        occupancy_queue_.pop();
        const int idx = Vox2Idx(xx.point);
        const int occupy = Exist(idx);
        const double log_odds_update =
            (num_hit_[idx] >= num_miss_[idx] - num_hit_[idx] ? prob_hit_log_
                                                             : prob_miss_log_);
        num_hit_[idx] = num_miss_[idx] = 0;
        if (distance_buffer_[idx] < 0.0) {
            distance_buffer_[idx] = static_cast<double>(kInfinity);
            InsertIntoList(reserved_idx_4_undefined_, idx);
        }
        if ((log_odds_update >= 0.0 && occupancy_buffer_[idx] >= clamp_max_log_) ||
            (log_odds_update <= 0.0 && occupancy_buffer_[idx] <= clamp_min_log_)) {
            continue;
        }
        if (!global_map && !VoxInRange(xx.point, false)) {
            occupancy_buffer_[idx] = 0.0;
            distance_buffer_[idx] = static_cast<double>(kInfinity);
        }
        occupancy_buffer_[idx] = std::min(
            std::max(occupancy_buffer_[idx] + log_odds_update, clamp_min_log_),
            clamp_max_log_);
        if (Exist(idx) && !occupy) {
            insert_queue_.push(QueueElement{xx.point, 0.0});
        } else if (!Exist(idx) && occupy) {
            delete_queue_.push(
                QueueElement{xx.point, static_cast<double>(kInfinity)});
        }
    }
    return !insert_queue_.empty() || !delete_queue_.empty();
}

void EsdfMap::UpdateESDF() {
    while (!insert_queue_.empty()) {
        const QueueElement xx = insert_queue_.front();
        insert_queue_.pop();
        const int idx = Vox2Idx(xx.point);
        if (!Exist(idx)) {
            continue;
        }
        DeleteFromList(Vox2Idx(closest_obstacle_[idx]), idx);
        closest_obstacle_[idx] = xx.point;
        distance_buffer_[idx] = 0.0;
        InsertIntoList(idx, idx);
        update_queue_.push(xx);
    }
    while (!delete_queue_.empty()) {
        const QueueElement xx = delete_queue_.front();
        delete_queue_.pop();
        const int idx = Vox2Idx(xx.point);
        if (Exist(idx)) {
            continue;
        }
        int next_obs_idx = kUndefined;
        for (int obs_idx = head_[idx]; obs_idx != kUndefined;
             obs_idx = next_obs_idx) {
            closest_obstacle_[obs_idx] =
                Eigen::Vector3i(kUndefined, kUndefined, kUndefined);
            const Eigen::Vector3i obs_vox = Idx2Vox(obs_idx);
            double distance = static_cast<double>(kInfinity);
            for (int d = 0; d < kNumDirs; ++d) {
                const Eigen::Vector3i new_pos = obs_vox + kDirs[d];
                const int new_pos_idx = Vox2Idx(new_pos);
                if (VoxInRange(new_pos) &&
                    closest_obstacle_[new_pos_idx](0) != kUndefined &&
                    Exist(Vox2Idx(closest_obstacle_[new_pos_idx]))) {
                    const double tmp = Dist(obs_vox, closest_obstacle_[new_pos_idx]);
                    if (tmp < distance) {
                        distance = tmp;
                        closest_obstacle_[obs_idx] = closest_obstacle_[new_pos_idx];
                    }
                    break;
                }
            }
            prev_[obs_idx] = kUndefined;
            next_obs_idx = next_[obs_idx];
            next_[obs_idx] = kUndefined;
            distance_buffer_[obs_idx] = distance;
            if (distance < static_cast<double>(kInfinity)) {
                update_queue_.push(QueueElement{obs_vox, distance});
            }
            InsertIntoList(Vox2Idx(closest_obstacle_[obs_idx]), obs_idx);
        }
        head_[idx] = kUndefined;
    }

    while (!update_queue_.empty()) {
        const QueueElement xx = update_queue_.front();
        update_queue_.pop();
        const int idx = Vox2Idx(xx.point);
        if (xx.distance != distance_buffer_[idx]) {
            continue;
        }
        bool change = false;
        for (int i = 0; i < kNumDirs; ++i) {
            const Eigen::Vector3i new_pos = xx.point + kDirs[i];
            if (!VoxInRange(new_pos)) {
                continue;
            }
            const int new_pos_idx = Vox2Idx(new_pos);
            if (closest_obstacle_[new_pos_idx](0) == kUndefined) {
                continue;
            }
            const double tmp = Dist(xx.point, closest_obstacle_[new_pos_idx]);
            if (StoredDistance(idx) > tmp) {
                distance_buffer_[idx] = tmp;
                change = true;
                DeleteFromList(Vox2Idx(closest_obstacle_[idx]), idx);
                InsertIntoList(Vox2Idx(closest_obstacle_[new_pos_idx]), idx);
                closest_obstacle_[idx] = closest_obstacle_[new_pos_idx];
            }
        }
        if (change) {
            update_queue_.push(QueueElement{xx.point, distance_buffer_[idx]});
            continue;
        }
        const int new_obs_idx = Vox2Idx(closest_obstacle_[idx]);
        for (int d = 0; d < kNumDirs; ++d) {
            const Eigen::Vector3i new_pos = xx.point + kDirs[d];
            if (!VoxInRange(new_pos)) {
                continue;
            }
            const int new_pos_id = Vox2Idx(new_pos);
            const double tmp = Dist(new_pos, closest_obstacle_[idx]);
            if (StoredDistance(new_pos_id) > tmp) {
                distance_buffer_[new_pos_id] = tmp;
                DeleteFromList(Vox2Idx(closest_obstacle_[new_pos_id]), new_pos_id);
                InsertIntoList(new_obs_idx, new_pos_id);
                closest_obstacle_[new_pos_id] = closest_obstacle_[idx];
                update_queue_.push(QueueElement{new_pos, tmp});
            }
        }
    }
}

void EsdfMap::SetUpdateRange(const Eigen::Vector3d& min_pos,
                             const Eigen::Vector3d& max_pos, bool new_vec) {
    Eigen::Vector3d lo = min_pos.cwiseMax(min_range_);
    Eigen::Vector3d hi = max_pos.cwiseMin(max_range_);
    if (new_vec) {
        last_min_vec_ = min_vec_;
        last_max_vec_ = max_vec_;
    }
    Pos2Vox(lo, &min_vec_);
    Pos2Vox(hi - Eigen::Vector3d::Constant(options_.resolution / 2.0),
            &max_vec_);
}

void EsdfMap::SetOriginalRange() {
    min_vec_ << 0, 0, 0;
    max_vec_ << grid_size_(0) - 1, grid_size_(1) - 1, grid_size_(2) - 1;
    last_min_vec_ = min_vec_;
    last_max_vec_ = max_vec_;
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
    const double res = options_.resolution;
    Raycast(origin / res, point / res, min_range_ / res, max_range_ / res,
            &voxels);
    const Eigen::Vector3d half(0.5, 0.5, 0.5);
    for (int i = static_cast<int>(voxels.size()) - 2; i >= 0; --i) {
        const Eigen::Vector3d world = (voxels[static_cast<std::size_t>(i)] + half) * res;
        const double along = (world - origin).norm();
        if (along < options_.min_ray_length) {
            break;
        }
        if (along > options_.max_ray_length) {
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
    for (int x = min_vec_(0); x <= max_vec_(0); ++x) {
        for (int y = min_vec_(1); y <= max_vec_(1); ++y) {
            for (int z = min_vec_(2); z <= max_vec_(2); ++z) {
                const Eigen::Vector3i vox(x, y, z);
                if (!Exist(Vox2Idx(vox))) {
                    continue;
                }
                Eigen::Vector3d pos;
                Vox2Pos(vox, &pos);
                points.push_back(pos);
            }
        }
    }
    return points;
}

bool EsdfMap::CheckConsistency() const {
    for (int x = 0; x < grid_size_(0); ++x) {
        for (int y = 0; y < grid_size_(1); ++y) {
            for (int z = 0; z < grid_size_(2); ++z) {
                const int vox = Vox2Idx(Eigen::Vector3i(x, y, z));
                if ((prev_[vox] != kUndefined && next_[prev_[vox]] != vox) ||
                    (next_[vox] != kUndefined && prev_[next_[vox]] != vox)) {
                    return false;
                }
                if (prev_[vox] == kUndefined && distance_buffer_[vox] >= 0.0 &&
                    head_[Vox2Idx(closest_obstacle_[vox])] != vox) {
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
