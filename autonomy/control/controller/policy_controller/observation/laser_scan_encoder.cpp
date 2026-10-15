/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 */

#include "autonomy/control/controller/policy_controller/observation/laser_scan_encoder.hpp"

#include <algorithm>
#include <cmath>

namespace autonomy {
namespace control {
namespace controller {
namespace policy_controller {

bool EncodeLaserScan(const ObservationSnapshot& snapshot,
                     const proto::ObservationInput& input,
                     int laser_scan_bin_count, std::vector<float>* output,
                     std::string* error) {
    if (output == nullptr) {
        return false;
    }
    if (snapshot.laser_scan == nullptr ||
        snapshot.laser_scan->ranges().empty()) {
        if (error != nullptr) {
            *error = "Laser-scan modality requires LaserScan ranges";
        }
        return false;
    }

    int bins = laser_scan_bin_count;
    if (input.shape_size() > 0) {
        for (int i = 0; i < input.shape_size(); ++i) {
            if (input.shape(i) > 1) {
                bins = static_cast<int>(input.shape(i));
            }
        }
    }
    bins = std::max(1, bins);
    output->assign(static_cast<std::size_t>(bins), 0.0f);

    const auto& ranges = snapshot.laser_scan->ranges();
    const float range_max = snapshot.laser_scan->range_max() > 0.0f
                                ? snapshot.laser_scan->range_max()
                                : 1.0f;
    const int source_count = ranges.size();
    for (int i = 0; i < bins; ++i) {
        const int source =
            source_count == 1
                ? 0
                : static_cast<int>(std::llround(
                      static_cast<double>(i) * (source_count - 1) /
                      static_cast<double>(bins - 1)));
        float range = ranges.Get(source);
        if (!std::isfinite(range) || range < 0.0f) {
            range = range_max;
        }
        range = std::min(range, range_max);
        (*output)[static_cast<std::size_t>(i)] = range / range_max;
    }
    return true;
}

}  // namespace policy_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
