#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "atlas/common/io/trajectory_io.hpp"
#include "atlas/sensor/sensor_types.hpp"

namespace autonomy::localization::atlas::examples {

/**
 * Records valid tracking poses by Atlas map and coordinate epoch. The selected
 * conventional TUM output is one map segment only; all raw segments and
 * their coordinate-frame boundaries are retained in sidecars.
 */
class TrajectorySegmentRecorder {
public:
    /** Records a result. Invalid poses do not split an unchanged map. */
    void Record(const TrackingResult& result);

    std::size_t valid_pose_count() const { return valid_pose_count_; }
    std::size_t segment_count() const { return segments_.size(); }

    /**
     * Writes path as either the active optimized segment or the longest raw
     * contiguous segment. Never concatenates independent reset frames.
     *
     * Each raw segment is written to path.segment-NNN.tum and a CSV manifest
     * to path.segments.csv. active_optimized_path may be empty or invalid.
     */
    bool SaveTum(const std::string& path,
                 const std::string& active_optimized_path) const;

private:
    struct Segment {
        std::uint64_t map_id = 0;
        bool map_id_valid = false;
        std::uint64_t coordinate_epoch = 0;
        std::uint64_t destructive_epoch = 0;
        std::vector<TrajectorySample> poses;
    };

    std::vector<Segment> segments_;
    std::size_t valid_pose_count_ = 0;
    bool break_before_next_valid_ = true;
    bool observed_map_id_valid_ = false;
    std::uint64_t observed_map_id_ = 0;
    std::uint64_t observed_coordinate_epoch_ = 0;
    std::uint64_t observed_destructive_epoch_ = 0;
};

}  // namespace autonomy::localization::atlas::examples
