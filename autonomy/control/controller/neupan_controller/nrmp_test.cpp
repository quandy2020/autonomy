/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 */

#include "autonomy/control/controller/neupan_controller/nrmp.hpp"

#include <cstdio>

namespace autonomy {
namespace control {
namespace controller {
namespace neupan_controller {

int Run() {
    const RobotModel robot = RobotModel::CreateRectangle(
        Kinematics::kDiff, 4, 0.1, Vec2(1.0, 1.0), Vec2(2.0, 2.0), 0.8, 0.6);
    NrmpParams params;
    params.max_num = 2;
    params.d_min = 0.05;
    params.d_max = 1.0;
    Nrmp nrmp(robot, params);

    Mat3X nom_s = Mat3X::Zero(3, 5);
    Mat2X nom_u = Mat2X::Zero(2, 4);
    Mat3X ref_s = Mat3X::Zero(3, 5);
    Vec ref_us = Vec::Constant(4, 0.4);
    for (int t = 0; t <= 4; ++t) {
        ref_s(0, t) = 0.04 * t;
        if (t > 0) {
            nom_s(0, t) = 0.04 * t;
        }
    }
    nom_u.row(0).setConstant(0.4);

    std::vector<Mat> fa;
    std::vector<Vec> fb;
    nrmp.BuildFaFb({}, {}, {}, &fa, &fb);
    const auto result = nrmp.Solve(nom_s, nom_u, ref_s, ref_us, fa, fb);
    if (!result.success) {
        std::fprintf(stderr, "FAIL: nrmp status=%d\n", result.status);
        return 1;
    }
    if (result.u(0, 0) <= 0.0 || std::abs(result.u(0, 0)) > 1.01) {
        std::fprintf(stderr, "FAIL: u0=%g\n", result.u(0, 0));
        return 1;
    }
    std::puts("neupan nrmp: ok");
    return 0;
}

}  // namespace neupan_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy

int main() {
    return autonomy::control::controller::neupan_controller::Run();
}
