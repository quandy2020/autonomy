/* Copyright 2026 The Openbot Authors. */
#include "autonomy/perception/observation/lidar_observation_adapter.hpp"

namespace autonomy::perception {
std::optional<SensorObservation> LidarObservationAdapter::Adapt(
    const LidarScan& s, const PoseLookup& poses) const {
    if (!poses || !s.localization_valid || s.beams.empty() || s.frame_id.empty() ||
        !std::isfinite(s.stamp) || !std::isfinite(s.duration) || s.duration < 0 ||
        !std::isfinite(s.valid_for) || s.valid_for <= 0 ||
        !std::isfinite(s.min_range) || !std::isfinite(s.max_range) ||
        s.min_range <= 0 || s.max_range <= s.min_range ||
        !std::isfinite(s.completeness) || s.completeness <= 0 || s.completeness > 1)
        return std::nullopt;
    SensorObservation o;
    o.sensor = SensorKind::kLidar; o.sequence = s.sequence; o.map_epoch = s.map_epoch;
    o.stamp = s.stamp + s.duration; o.valid_until = o.stamp + s.valid_for;
    o.localization_valid = true; o.frame_id = s.frame_id; o.scan_completeness = s.completeness;
    for (const auto& b : s.beams) {
        if (!b.valid || !b.direction.Finite() || std::abs(b.direction.Norm()-1) > 1e-5 ||
            !std::isfinite(b.range) || b.range < s.min_range || b.range > s.max_range ||
            !std::isfinite(b.time_offset) || b.time_offset < 0 || b.time_offset > s.duration ||
            !std::isfinite(b.quality) || b.quality <= 0 || b.quality > 1) continue;
        const double stamp = s.stamp + b.time_offset;
        auto pose = poses(stamp);
        if (!pose || !pose->Valid()) continue;
        o.rays.push_back({pose->translation, pose->Rotate(b.direction), b.range,
                          b.hit, b.quality, stamp});
    }
    o.valid_fraction = static_cast<double>(o.rays.size()) / s.beams.size();
    return o;
}
}  // namespace autonomy::perception
