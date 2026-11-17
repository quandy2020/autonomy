/* Copyright 2026 The Openbot Authors. */
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace autonomy::perception {

enum class SensorKind : std::uint8_t { kLidar = 1, kRgbd = 2 };
enum class SensorMode : std::uint8_t { kLidarOnly = 1, kRgbdOnly = 2, kLidarRgbd = 3 };
using SensorMask = std::uint8_t;
constexpr SensorMask Mask(SensorKind sensor) { return static_cast<SensorMask>(sensor); }
constexpr SensorMask Mask(SensorMode mode) { return static_cast<SensorMask>(mode); }

struct ObservationCut {
    std::array<std::uint64_t, 2> sequence{0, 0};
    SensorMask present = 0;
    void Include(SensorKind sensor, std::uint64_t value) {
        const auto index = sensor == SensorKind::kLidar ? 0U : 1U;
        sequence[index] = std::max(sequence[index], value);
        present |= Mask(sensor);
    }
    bool Covers(const ObservationCut& other) const {
        if ((present & other.present) != other.present) return false;
        for (unsigned i = 0; i < 2; ++i)
            if ((other.present & (1U << i)) && sequence[i] < other.sequence[i]) return false;
        return true;
    }
    bool operator==(const ObservationCut& other) const {
        return present == other.present && sequence == other.sequence;
    }
};

struct Vec3 {
    double x = 0.0, y = 0.0, z = 0.0;
    Vec3 operator+(const Vec3& b) const { return {x + b.x, y + b.y, z + b.z}; }
    Vec3 operator-(const Vec3& b) const { return {x - b.x, y - b.y, z - b.z}; }
    Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    double Norm() const { return std::sqrt(x*x + y*y + z*z); }
    bool Finite() const { return std::isfinite(x) && std::isfinite(y) && std::isfinite(z); }
};

struct RigidTransform {
    // Row-major rotation; camera optical +Z is forward. No implicit axis swap.
    std::array<double, 9> rotation{1, 0, 0, 0, 1, 0, 0, 0, 1};
    Vec3 translation;
    Vec3 Rotate(const Vec3& p) const {
        return {rotation[0]*p.x + rotation[1]*p.y + rotation[2]*p.z,
                rotation[3]*p.x + rotation[4]*p.y + rotation[5]*p.z,
                rotation[6]*p.x + rotation[7]*p.y + rotation[8]*p.z};
    }
    Vec3 Apply(const Vec3& p) const { return Rotate(p) + translation; }
    bool Valid() const;
};

struct CameraModel {
    int width = 0, height = 0;
    double fx = 0, fy = 0, cx = 0, cy = 0;
    double min_depth = 0.1, max_depth = 8.0;
    bool Valid() const;
};

struct ObservationRay {
    Vec3 origin, direction;  // Unit direction, world frame.
    double distance = 0.0;  // Actual measured ray range, meters.
    bool hit = true;
    double quality = 1.0;  // Geometric quality weight, not probability.
    double stamp = 0.0;    // Individual LiDAR beam acquisition time.
    Vec3 Endpoint() const { return origin + direction * distance; }
};

struct SensorObservation {
    SensorKind sensor = SensorKind::kRgbd;
    std::uint64_t sequence = 0, map_epoch = 0;
    std::string frame_id = "map";
    double stamp = 0.0, valid_until = 0.0;
    double valid_fraction = 0.0, scan_completeness = 1.0;
    bool localization_valid = false;
    std::vector<ObservationRay> rays;
    bool Valid(double now) const;
};

struct DepthView {
    SensorKind sensor = SensorKind::kRgbd;
    std::uint64_t sequence = 0, map_epoch = 0;
    std::uint32_t view_id = 0;
    double stamp = 0.0;
    CameraModel camera;
    RigidTransform world_from_camera;
    std::vector<float> axial_depth;   // Optical Z, meters. Zero only means invalid.
    std::vector<std::uint8_t> valid; // Never infer free space from missing pixels.
};

}  // namespace autonomy::perception
