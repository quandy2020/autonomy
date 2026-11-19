/* Copyright 2026 The Openbot Authors. */
#pragma once
#include <optional>
#include "autonomy/perception/data/sensor_observation.hpp"

namespace autonomy::perception {
struct RgbdFrame {
    CameraModel camera;
    std::vector<float> depth;  // Native units, converted once with depth_scale.
    double depth_scale = 1.0;
    RigidTransform world_from_camera;
    std::uint64_t sequence = 0, map_epoch = 0;
    double stamp = 0.0, valid_for = 0.5;
    bool localization_valid = false;
    std::string frame_id = "map";
};
class RgbdObservationAdapter {
public:
    /** @brief Generate rays only from the explicitly selected actual valid pixels.
     * @details valid_fraction describes the full actual raster, independent from
     * the ray work budget; unsampled pixels do not become no-hit free-space rays.
     */
    std::optional<SensorObservation> Adapt(const RgbdFrame& frame, unsigned pixel_stride = 1) const;
    std::optional<DepthView> MakeDepthView(const RgbdFrame& frame) const;
};
}  // namespace autonomy::perception
