/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

#include "atlas/map/dense_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace autonomy::localization::atlas::map {
namespace {

std::uint64_t Mix(std::uint64_t value) {
    value ^= value >> 30U;
    value *= 0xbf58476d1ce4e5b9ULL;
    value ^= value >> 27U;
    value *= 0x94d049bb133111ebULL;
    return value ^ (value >> 31U);
}

std::uint8_t RoundColor(float value) {
    return static_cast<std::uint8_t>(
        std::lround(std::clamp(value, 0.0F, 255.0F)));
}

}  // namespace

DenseMap::DenseMap(DenseMapOptions options) : options_(std::move(options)) {
    if (!std::isfinite(options_.voxel_size) || options_.voxel_size <= 0.0 ||
        options_.maximum_voxels == 0) {
        throw std::invalid_argument("invalid dense-map options");
    }
    voxels_.reserve(
        std::min(options_.reserve_voxels, options_.maximum_voxels));
    ResetPriorityIndex();
}

std::size_t DenseMap::VoxelKeyHash::operator()(const VoxelKey& key) const {
    const auto x = static_cast<std::uint32_t>(key.x);
    const auto y = static_cast<std::uint32_t>(key.y);
    const auto z = static_cast<std::uint32_t>(key.z);
    return static_cast<std::size_t>(Mix(x) ^ (Mix(y) << 1U) ^
                                    (Mix(z) << 2U));
}

std::uint64_t DenseMap::PriorityForKey(const VoxelKey& key) {
    const auto x = static_cast<std::uint32_t>(key.x);
    const auto y = static_cast<std::uint32_t>(key.y);
    const auto z = static_cast<std::uint32_t>(key.z);
    const std::uint64_t combined =
        Mix(static_cast<std::uint64_t>(x) ^ 0x243f6a8885a308d3ULL) ^
        Mix(static_cast<std::uint64_t>(y) ^ 0x13198a2e03707344ULL) ^
        Mix(static_cast<std::uint64_t>(z) ^ 0xa4093822299f31d0ULL);
    return Mix(combined);
}

void DenseMap::ResetPriorityIndex() {
    PriorityStorage storage;
    storage.reserve(
        std::min(options_.reserve_voxels, options_.maximum_voxels));
    retained_priorities_ =
        PriorityQueue(PriorityEntryLess{}, std::move(storage));
}

void DenseMap::RebuildPriorityIndex() {
    PriorityStorage storage;
    storage.reserve(voxels_.size());
    for (const auto& [key, value] : voxels_) {
        static_cast<void>(value);
        storage.push_back(PriorityEntry{PriorityForKey(key), key});
    }
    // The priority_queue constructor heapifies in linear time; repeated push
    // would make every rolling-window recenter O(N log N).
    retained_priorities_ =
        PriorityQueue(PriorityEntryLess{}, std::move(storage));
}

bool DenseMap::KeyForPoint(const Eigen::Vector3f& point, VoxelKey* key) const {
    if (key == nullptr || !point.allFinite())
        return false;
    const double inverse = 1.0 / options_.voxel_size;
    const double x = std::floor(static_cast<double>(point.x()) * inverse);
    const double y = std::floor(static_cast<double>(point.y()) * inverse);
    const double z = std::floor(static_cast<double>(point.z()) * inverse);
    constexpr double kMinimum =
        static_cast<double>(std::numeric_limits<std::int32_t>::min());
    constexpr double kMaximum =
        static_cast<double>(std::numeric_limits<std::int32_t>::max());
    if (x < kMinimum || x > kMaximum || y < kMinimum || y > kMaximum ||
        z < kMinimum || z > kMaximum) {
        return false;
    }
    key->x = static_cast<std::int32_t>(x);
    key->y = static_cast<std::int32_t>(y);
    key->z = static_cast<std::int32_t>(z);
    return true;
}

std::size_t DenseMap::Integrate(
    const std::vector<ColorPoint, Eigen::aligned_allocator<ColorPoint>>&
        points) {
    std::size_t accepted = 0;
    for (const ColorPoint& point : points) {
        VoxelKey key;
        if (!KeyForPoint(point.position, &key))
            continue;
        auto iterator = voxels_.find(key);
        if (iterator == voxels_.end()) {
            const PriorityEntry candidate{PriorityForKey(key), key};
            if (voxels_.size() >= options_.maximum_voxels) {
                const PriorityEntry& worst = retained_priorities_.top();
                if (!PriorityEntryLess{}(candidate, worst))
                    continue;
                voxels_.erase(worst.key);
                retained_priorities_.pop();
            }
            VoxelValue value;
            value.mean = point.position;
            value.mean_red = point.red;
            value.mean_green = point.green;
            value.mean_blue = point.blue;
            value.observations = 1;
            voxels_.emplace(key, value);
            retained_priorities_.push(candidate);
            ++accepted;
            continue;
        }

        VoxelValue& value = iterator->second;
        if (value.observations == std::numeric_limits<std::uint32_t>::max()) {
            continue;
        }
        ++value.observations;
        const float weight = 1.0F / static_cast<float>(value.observations);
        value.mean += (point.position - value.mean) * weight;
        value.mean_red += (static_cast<float>(point.red) - value.mean_red) *
                          weight;
        value.mean_green +=
            (static_cast<float>(point.green) - value.mean_green) * weight;
        value.mean_blue +=
            (static_cast<float>(point.blue) - value.mean_blue) * weight;
        ++accepted;
    }
    return accepted;
}

std::size_t DenseMap::RemoveOutsideRadius(const Eigen::Vector3f& center,
                                         double radius) {
    if (!center.allFinite() || !std::isfinite(radius) || radius < 0.0)
        return 0;

    const Eigen::Vector3d center_double = center.cast<double>();
    const double squared_radius = radius * radius;
    std::size_t removed = 0;
    for (auto iterator = voxels_.begin(); iterator != voxels_.end();) {
        const Eigen::Vector3d delta =
            iterator->second.mean.cast<double>() - center_double;
        if (!delta.allFinite() || delta.squaredNorm() > squared_radius) {
            iterator = voxels_.erase(iterator);
            ++removed;
        } else {
            ++iterator;
        }
    }
    if (removed > 0)
        RebuildPriorityIndex();
    return removed;
}

DenseMapSnapshot DenseMap::Snapshot(double timestamp,
                                    std::uint64_t generation) const {
    DenseMapSnapshot snapshot;
    snapshot.timestamp = timestamp;
    snapshot.generation = generation;
    snapshot.points.reserve(voxels_.size());
    for (const auto& [key, value] : voxels_) {
        static_cast<void>(key);
        ColorPoint point;
        point.position = value.mean;
        point.red = RoundColor(value.mean_red);
        point.green = RoundColor(value.mean_green);
        point.blue = RoundColor(value.mean_blue);
        snapshot.points.push_back(point);
    }
    return snapshot;
}

void DenseMap::Reset() {
    voxels_.clear();
    voxels_.reserve(
        std::min(options_.reserve_voxels, options_.maximum_voxels));
    ResetPriorityIndex();
}

}  // namespace autonomy::localization::atlas::map
