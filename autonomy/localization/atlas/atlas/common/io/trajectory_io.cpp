#include "atlas/common/io/trajectory_io.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>

namespace autonomy::localization::atlas {
namespace {

bool Valid(const TrajectorySample& sample) {
    return std::isfinite(sample.timestamp) &&
           sample.world_from_camera.matrix().allFinite();
}

}  // namespace

bool TrajectoryIO::SaveTum(const std::string& path,
                           const std::vector<TrajectorySample>& trajectory) {
    std::ofstream output(path);
    if (!output)
        return false;
    output << std::fixed;
    for (const TrajectorySample& sample : trajectory) {
        if (!Valid(sample))
            return false;
        const Eigen::Vector3f translation =
            sample.world_from_camera.translation();
        const Eigen::Quaternionf rotation =
            sample.world_from_camera.unit_quaternion();
        output << std::setprecision(9) << sample.timestamp << ' '
               << std::setprecision(7) << translation.x() << ' '
               << translation.y() << ' ' << translation.z() << ' '
               << rotation.x() << ' ' << rotation.y() << ' ' << rotation.z()
               << ' ' << rotation.w() << '\n';
    }
    return static_cast<bool>(output);
}

bool TrajectoryIO::SaveEuroc(const std::string& path,
                             const std::vector<TrajectorySample>& trajectory) {
    std::ofstream output(path);
    if (!output)
        return false;
    output << std::fixed;
    for (const TrajectorySample& sample : trajectory) {
        if (!Valid(sample))
            return false;
        const Eigen::Vector3f translation =
            sample.world_from_camera.translation();
        const Eigen::Quaternionf rotation =
            sample.world_from_camera.unit_quaternion();
        const auto nanoseconds =
            static_cast<long long>(std::llround(sample.timestamp * 1.0e9));
        output << nanoseconds << ',' << std::setprecision(7) << translation.x()
               << ',' << translation.y() << ',' << translation.z() << ','
               << rotation.w() << ',' << rotation.x() << ',' << rotation.y()
               << ',' << rotation.z() << '\n';
    }
    return static_cast<bool>(output);
}

}  // namespace autonomy::localization::atlas
