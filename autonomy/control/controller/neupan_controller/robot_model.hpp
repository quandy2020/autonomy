/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

/**
 * @file robot_model.hpp
 * @brief NeuPAN kinematics, convex footprint halfspaces, and linearized dynamics.
 */

#include "autonomy/control/controller/neupan_controller/types.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace neupan_controller {

class RobotModel
{
public:
    RobotModel() = default;
    RobotModel(Kinematics kinematics, int receding, double step_time,
               Vec2 max_speed, Vec2 max_acce, const Mat2X& vertices);

    static RobotModel CreateRectangle(Kinematics kinematics, int receding,
                                      double step_time, Vec2 max_speed,
                                      Vec2 max_acce, double length,
                                      double width, double wheelbase = 0.0);

    static Mat2X RectangleVertices(Kinematics kinematics, double length,
                                   double width, double wheelbase);

    void Linearize(const Vec3& nom_s, const Vec2& nom_u, Mat33* A, Mat32* B,
                   Vec3* C) const;

    int EdgeCount() const {
        return static_cast<int>(h.size());
    }

    Kinematics kinematics{Kinematics::kDiff};
    int T{10};
    double dt{0.1};
    Vec2 max_speed{1.0, 1.0};
    Vec2 acce_bound{1.0, 1.0};
    Mat G;
    Vec h;
};

}  // namespace neupan_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
