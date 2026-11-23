/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 */

#include "autonomy/control/controller/neupan_controller/planner.hpp"

#include <cstdio>

namespace autonomy {
namespace control {
namespace controller {
namespace neupan_controller {

int Run() {
    NeuPanPlanner::Config config;
    config.receding = 4;
    config.step_time = 0.1;
    config.ref_speed = 0.5;
    config.collision_threshold = 0.05;
    config.max_speed = Vec2(1.0, 1.0);
    config.max_acce = Vec2(2.0, 2.0);
    config.adjust.max_num = 2;
    NeuPanPlanner planner(config, nullptr);

    std::vector<InitialPath::PathPoint> path;
    path.emplace_back(0.0, 0.0, 0.0, 1.0);
    path.emplace_back(3.0, 0.0, 0.0, 1.0);
    planner.SetInitialPath(path);

    NeuPanPlanner::Info info;
    const Vec2 u = planner.Forward(Vec3(0.0, 0.0, 0.0), Mat2X(2, 0), &info);
    if (info.arrive || info.stop || u(0) <= 0.0) {
        std::fprintf(stderr, "FAIL: planner u=(%g,%g) arrive=%d stop=%d solved=%d\n",
                     u(0), u(1), info.arrive, info.stop, info.solved);
        return 1;
    }
    std::puts("neupan planner: ok");
    return 0;
}

}  // namespace neupan_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy

int main() {
    return autonomy::control::controller::neupan_controller::Run();
}
