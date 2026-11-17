/* Copyright 2026 The Openbot Authors. */
#pragma once
#include <functional>
#include <optional>
#include "autonomy/perception/data/sensor_observation.hpp"

namespace autonomy::perception {
struct LidarBeam {
    Vec3 direction;
    double range = 0.0, time_offset = 0.0;
    bool valid = false, hit = true;
    double quality = 1.0;
};
struct LidarScan {
    std::uint64_t sequence = 0, map_epoch = 0;
    double stamp = 0.0, duration = 0.0, valid_for = 0.5;
    double min_range = 0.1, max_range = 30.0;
    double completeness = 1.0;
    std::string frame_id = "map";
    bool localization_valid = false;
    std::vector<LidarBeam> beams;
};
class LidarObservationAdapter {
public:
    // Caller provides timestamped world_from_lidar from its localization chain.
    // Missing transform rejects that beam; no whole-scan rigid-time assumption.
    using PoseLookup = std::function<std::optional<RigidTransform>(double)>;
    std::optional<SensorObservation> Adapt(const LidarScan& scan, const PoseLookup& poses) const;
};
}  // namespace autonomy::perception
