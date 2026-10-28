#pragma once

#include <string>
#include <vector>

#include "autonomy/common/math/se3.hpp"

namespace autonomy::localization::atlas {

/** @brief One timestamped world-from-camera trajectory pose. */
struct TrajectorySample {
    double timestamp = 0.0;
    autonomy::common::math::SE3f world_from_camera;
};

/** @brief Writers for common visual-SLAM trajectory interchange formats. */
class TrajectoryIO
{
public:
    /** @brief Writes timestamp, translation and quaternion in TUM format. */
    static bool SaveTum(const std::string& path,
                        const std::vector<TrajectorySample>& trajectory);
    /** @brief Writes nanosecond timestamps and body pose in EuRoC format. */
    static bool SaveEuroc(const std::string& path,
                          const std::vector<TrajectorySample>& trajectory);
};

}  // namespace autonomy::localization::atlas
