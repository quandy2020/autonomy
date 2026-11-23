/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 */

#include "autonomy/control/controller/neupan_controller/robot_model.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace autonomy {
namespace control {
namespace controller {
namespace neupan_controller {
namespace {

int Fail(const char* msg) {
    std::fprintf(stderr, "FAIL: %s\n", msg);
    return 1;
}

}  // namespace

int Run() {
    const RobotModel robot = RobotModel::CreateRectangle(
        Kinematics::kDiff, 5, 0.1, Vec2(1.0, 1.0), Vec2(1.0, 1.0), 0.8, 0.6);
    if (robot.T != 5 || robot.G.rows() != 4 || robot.h.size() != 4) {
        return Fail("rectangle halfspaces");
    }
    if (std::fabs(robot.acce_bound(0) - 0.1) > 1e-9) {
        return Fail("acce_bound");
    }
    Mat33 A;
    Mat32 B;
    Vec3 C;
    robot.Linearize(Vec3(0, 0, 0), Vec2(0, 0), &A, &B, &C);
    if (!A.isIdentity(1e-12) || std::fabs(B(0, 0) - 0.1) > 1e-12 ||
        std::fabs(B(2, 1) - 0.1) > 1e-12 || C.norm() > 1e-12) {
        return Fail("linearize at rest");
    }
    std::puts("neupan robot_model: ok");
    return 0;
}

}  // namespace neupan_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy

int main() {
    return autonomy::control::controller::neupan_controller::Run();
}
