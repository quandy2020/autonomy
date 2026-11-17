/* Copyright 2026 The Openbot Authors. */
#include "autonomy/perception/observation/rgbd_observation_adapter.hpp"

namespace autonomy::perception {
std::optional<DepthView> RgbdObservationAdapter::MakeDepthView(const RgbdFrame& f) const {
    if (!f.camera.Valid() || !f.world_from_camera.Valid() || !f.localization_valid ||
        !std::isfinite(f.stamp) || !std::isfinite(f.valid_for) || f.valid_for <= 0 ||
        !std::isfinite(f.depth_scale) || f.depth_scale <= 0 || f.frame_id.empty() ||
        f.depth.size() != static_cast<std::size_t>(f.camera.width)*f.camera.height)
        return std::nullopt;
    DepthView view;
    view.camera = f.camera; view.world_from_camera = f.world_from_camera;
    view.sequence = f.sequence; view.map_epoch = f.map_epoch; view.stamp = f.stamp;
    view.axial_depth.resize(f.depth.size(), 0.0F); view.valid.resize(f.depth.size(), 0);
    for (std::size_t i = 0; i < f.depth.size(); ++i) {
        const double z = f.depth[i] * f.depth_scale;
        if (std::isfinite(z) && z >= f.camera.min_depth && z <= f.camera.max_depth) {
            view.axial_depth[i] = static_cast<float>(z); view.valid[i] = 1;
        }
    }
    return view;
}
std::optional<SensorObservation> RgbdObservationAdapter::Adapt(const RgbdFrame& f) const {
    auto view = MakeDepthView(f);
    if (!view) return std::nullopt;
    SensorObservation o;
    o.sensor = SensorKind::kRgbd; o.sequence = f.sequence; o.map_epoch = f.map_epoch;
    o.stamp = f.stamp; o.valid_until = f.stamp + f.valid_for;
    o.localization_valid = f.localization_valid; o.frame_id = f.frame_id;
    for (int y = 0; y < f.camera.height; ++y) for (int x = 0; x < f.camera.width; ++x) {
        const auto i = static_cast<std::size_t>(y)*f.camera.width+x;
        if (!view->valid[i]) continue;
        const double z = view->axial_depth[i];
        const Vec3 p{(x-f.camera.cx)*z/f.camera.fx, (y-f.camera.cy)*z/f.camera.fy, z};
        const double range = p.Norm();
        o.rays.push_back({f.world_from_camera.translation,
                          f.world_from_camera.Rotate(p*(1.0/range)), range,
                          true, 1.0, f.stamp});
    }
    o.valid_fraction = static_cast<double>(o.rays.size()) / f.depth.size();
    return o;
}
}  // namespace autonomy::perception
