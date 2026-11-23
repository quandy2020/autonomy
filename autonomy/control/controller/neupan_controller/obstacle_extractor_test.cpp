/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 */

#include "autonomy/control/controller/neupan_controller/obstacle_extractor.hpp"

#include <cmath>
#include <cstdio>
#include <limits>

namespace autonomy {
namespace control {
namespace controller {
namespace neupan_controller {

int Run() {
    automsgs::msgs::sensor_msgs::LaserScan scan;
    scan.set_angle_min(0.0);
    scan.set_angle_increment(M_PI / 2.0);
    scan.add_ranges(1.0);
    scan.add_ranges(2.0);
    scan.add_ranges(std::numeric_limits<float>::infinity());
    scan.add_ranges(0.5);

    LaserExtrinsic extrinsic;
    extrinsic.range_min = 0.1;
    extrinsic.range_max = 10.0;
    Eigen::Matrix2Xd points;
    const int n =
        ExtractObstaclePointsFromLaserScan(scan, extrinsic, 10, &points);
    if (n != 3 || points.cols() != 3 || std::fabs(points(0, 0) - 1.0) > 1e-9) {
        std::fprintf(stderr, "FAIL: n=%d cols=%ld p00=%g\n", n,
                     static_cast<long>(points.cols()),
                     points.cols() ? points(0, 0) : 0.0);
        return 1;
    }
    std::puts("neupan obstacle_extractor: ok");
    return 0;
}

}  // namespace neupan_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy

int main() {
    return autonomy::control::controller::neupan_controller::Run();
}
