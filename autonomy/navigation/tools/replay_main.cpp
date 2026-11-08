/* Copyright 2026 The Openbot Authors (duyongquan)
 * Licensed under the Apache License, Version 2.0.
 */
/** @file tools/replay_main.cpp
 * @brief Navigation contract, implementation or regression fixture.
 */
#include <iostream>
#include <sstream>
#include "navigation/planning/runner.hpp"
int main(int argc, char** argv) {
    using namespace autonomy::navigation;
    if (argc != 2) {
        std::cerr << "usage: navigation_replay CONFIG_DIRECTORY < "
                     "commands.txt\nrows: now "
                     "dt sensor_stamp v w measured_v measured_w coverage "
                     "obstacle_count [x y vx vy radius]...\n";
        return 2;
    }
    std::string error;
    auto config = LoadConfig(argv[1], &error);
    if (!config) {
        std::cerr << error << '\n';
        return 2;
    }
    SafetyShield shield(config->safety);
    std::string line;
    while (std::getline(std::cin, line)) {
        std::istringstream row(line);
        double now, dt;
        Twist desired, measured;
        ReflexObservation near;
        size_t count;
        if (!(row >> now >> dt >> near.stamp >> desired.linear >>
              desired.angular >> measured.linear >> measured.angular >>
              near.coverage_radius >> count) ||
            count > config->safety.max_obstacles) {
            std::cerr << "invalid replay row\n";
            return 1;
        }
        near.valid = true;
        for (size_t i = 0; i < count; ++i) {
            Obstacle obstacle;
            if (!(row >> obstacle.position.x() >> obstacle.position.y() >>
                  obstacle.velocity.x() >> obstacle.velocity.y() >>
                  obstacle.radius))
                return 1;
            near.obstacles.push_back(obstacle);
        }
        row >> std::ws;
        if (!row.eof())
            return 1;
        auto result = shield.Process(desired, near, measured, now, dt);
        std::cout << now << ' ' << result.command.linear << ' '
                  << result.command.angular << ' ' << result.emergency << ' '
                  << result.reason << '\n';
    }
    return std::cin.bad() ? 1 : 0;
}
