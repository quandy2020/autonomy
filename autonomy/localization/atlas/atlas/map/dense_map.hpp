/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <queue>
#include <unordered_map>
#include <vector>

#include "atlas/map/map_types.hpp"

namespace autonomy::localization::atlas::map {

/** Incremental coloured point map backed by a fixed-resolution voxel hash. */
class DenseMap {
public:
    explicit DenseMap(DenseMapOptions options);

    DenseMap(const DenseMap&) = delete;
    DenseMap& operator=(const DenseMap&) = delete;
    DenseMap(DenseMap&&) noexcept = default;
    DenseMap& operator=(DenseMap&&) noexcept = default;

    /** Fuses finite map-frame points and returns the number accepted. */
    std::size_t Integrate(
        const std::vector<ColorPoint, Eigen::aligned_allocator<ColorPoint>>&
            points);
    /** Removes voxels outside a sphere and rebuilds the capacity index. */
    std::size_t RemoveOutsideRadius(const Eigen::Vector3f& center,
                                    double radius);
    DenseMapSnapshot Snapshot(double timestamp, std::uint64_t generation) const;
    void Reset();

    std::size_t size() const { return voxels_.size(); }
    const DenseMapOptions& options() const { return options_; }

private:
    struct VoxelKey {
        std::int32_t x = 0;
        std::int32_t y = 0;
        std::int32_t z = 0;

        bool operator==(const VoxelKey& other) const {
            return x == other.x && y == other.y && z == other.z;
        }
    };

    struct VoxelKeyHash {
        std::size_t operator()(const VoxelKey& key) const;
    };

    struct VoxelValue {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Eigen::Vector3f mean = Eigen::Vector3f::Zero();
        float mean_red = 0.0F;
        float mean_green = 0.0F;
        float mean_blue = 0.0F;
        std::uint32_t observations = 0;
    };

    struct PriorityEntry {
        std::uint64_t priority = 0;
        VoxelKey key;
    };

    /** Orders the worst retained stable priority at the top of the heap. */
    struct PriorityEntryLess {
        bool operator()(const PriorityEntry& left,
                        const PriorityEntry& right) const {
            if (left.priority != right.priority)
                return left.priority < right.priority;
            if (left.key.x != right.key.x)
                return left.key.x < right.key.x;
            if (left.key.y != right.key.y)
                return left.key.y < right.key.y;
            return left.key.z < right.key.z;
        }
    };

    using PriorityStorage = std::vector<PriorityEntry>;
    using PriorityQueue = std::priority_queue<
        PriorityEntry, PriorityStorage, PriorityEntryLess>;

    bool KeyForPoint(const Eigen::Vector3f& point, VoxelKey* key) const;
    static std::uint64_t PriorityForKey(const VoxelKey& key);
    void ResetPriorityIndex();
    void RebuildPriorityIndex();

    DenseMapOptions options_;
    std::unordered_map<VoxelKey, VoxelValue, VoxelKeyHash> voxels_;
    PriorityQueue retained_priorities_;
};

}  // namespace autonomy::localization::atlas::map
