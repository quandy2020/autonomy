/* Copyright 2026 The Openbot Authors. */
#include "autonomy/perception/data/sensor_observation.hpp"

namespace autonomy::perception {
bool RigidTransform::Valid() const {
    if (!translation.Finite()) return false;
    for (double value : rotation) if (!std::isfinite(value)) return false;
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) {
        double dot = 0.0;
        for (int k = 0; k < 3; ++k) dot += rotation[3*i+k] * rotation[3*j+k];
        if (std::abs(dot - (i == j ? 1.0 : 0.0)) > 1e-5) return false;
    }
    const auto& r = rotation;
    double det = r[0]*(r[4]*r[8]-r[5]*r[7]) - r[1]*(r[3]*r[8]-r[5]*r[6])
                 + r[2]*(r[3]*r[7]-r[4]*r[6]);
    return std::abs(det - 1.0) < 1e-5;
}
bool CameraModel::Valid() const {
    return width > 0 && height > 0 && width <= 16384 && height <= 16384 &&
           std::isfinite(fx) && fx > 0 && std::isfinite(fy) && fy > 0 &&
           std::isfinite(cx) && std::isfinite(cy) && cx >= 0 && cy >= 0 &&
           cx < width && cy < height && std::isfinite(min_depth) &&
           std::isfinite(max_depth) && min_depth > 0 && max_depth > min_depth;
}
bool SensorObservation::Valid(double now) const {
    if (!std::isfinite(now) || !std::isfinite(stamp) || stamp > now + 1e-6 ||
        !std::isfinite(valid_until) || valid_until < now || valid_until < stamp ||
        !localization_valid || frame_id.empty() || rays.empty() ||
        !std::isfinite(valid_fraction) || valid_fraction <= 0 || valid_fraction > 1 ||
        !std::isfinite(scan_completeness) || scan_completeness <= 0 || scan_completeness > 1)
        return false;
    for (const auto& ray : rays) {
        if (!ray.origin.Finite() || !ray.direction.Finite() ||
            std::abs(ray.direction.Norm() - 1.0) > 1e-5 ||
            !std::isfinite(ray.distance) || ray.distance <= 0 ||
            !std::isfinite(ray.quality) || ray.quality <= 0 || ray.quality > 1 ||
            !std::isfinite(ray.stamp) || ray.stamp > now + 1e-6 || ray.stamp > valid_until)
            return false;
    }
    return true;
}
}  // namespace autonomy::perception
