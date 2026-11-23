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

#include "autonomy/control/controller/neupan_controller/robot_model.hpp"

#include <cmath>

namespace autonomy {
namespace control {
namespace controller {
namespace neupan_controller {
namespace {

void HalfspacesFromVertices(const Mat2X& vertices, Mat* G, Vec* h) {
    const int n = static_cast<int>(vertices.cols());
    if (n < 3) {
        G->resize(0, 2);
        h->resize(0);
        return;
    }
    G->resize(n, 2);
    h->resize(n);
    for (int i = 0; i < n; ++i) {
        const Vec2 p0 = vertices.col(i);
        const Vec2 p1 = vertices.col((i + 1) % n);
        const double ex = p1.x() - p0.x();
        const double ey = p1.y() - p0.y();
        (*G)(i, 0) = ey;
        (*G)(i, 1) = -ex;
        (*h)(i) = ey * p0.x() - ex * p0.y();
    }
}

}  // namespace

Mat2X RobotModel::RectangleVertices(Kinematics kinematics, double length,
                                    double width, double wheelbase) {
    const double half_width = 0.5 * width;
    Mat2X v(2, 4);
    if (kinematics != Kinematics::kOmni && wheelbase > 0.0) {
        const double rear = 0.5 * (length - wheelbase);
        const double front = length - rear;
        v.col(0) << -rear, -half_width;
        v.col(1) << front, -half_width;
        v.col(2) << front, half_width;
        v.col(3) << -rear, half_width;
    } else {
        const double half_length = 0.5 * length;
        v.col(0) << -half_length, -half_width;
        v.col(1) << half_length, -half_width;
        v.col(2) << half_length, half_width;
        v.col(3) << -half_length, half_width;
    }
    return v;
}

RobotModel::RobotModel(Kinematics kinematics_in, int receding, double step_time,
                       Vec2 max_speed_in, Vec2 max_acce, const Mat2X& vertices)
    : kinematics(kinematics_in),
      T(receding),
      dt(step_time),
      max_speed(std::move(max_speed_in)),
      acce_bound(max_acce * step_time) {
    HalfspacesFromVertices(vertices, &G, &h);
}

RobotModel RobotModel::CreateRectangle(Kinematics kinematics, int receding,
                                       double step_time, Vec2 max_speed,
                                       Vec2 max_acce, double length,
                                       double width, double wheelbase) {
    return RobotModel(kinematics, receding, step_time, std::move(max_speed),
                      std::move(max_acce),
                      RectangleVertices(kinematics, length, width, wheelbase));
}

void RobotModel::Linearize(const Vec3& nom_s, const Vec2& nom_u, Mat33* A,
                           Mat32* B, Vec3* C) const {
    const double phi = nom_s(2);
    const double v = nom_u(0);
    if (kinematics == Kinematics::kOmni) {
        const double psi = nom_u(1);
        A->setIdentity();
        *B << std::cos(psi) * dt, -v * std::sin(psi) * dt, std::sin(psi) * dt,
            v * std::cos(psi) * dt, 0.0, 0.0;
        *C << psi * v * std::sin(psi) * dt, -psi * v * std::cos(psi) * dt, 0.0;
        return;
    }
    *A << 1.0, 0.0, -v * dt * std::sin(phi), 0.0, 1.0, v * dt * std::cos(phi),
        0.0, 0.0, 1.0;
    *B << std::cos(phi) * dt, 0.0, std::sin(phi) * dt, 0.0, 0.0, dt;
    *C << phi * v * std::sin(phi) * dt, -phi * v * std::cos(phi) * dt, 0.0;
}

}  // namespace neupan_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
