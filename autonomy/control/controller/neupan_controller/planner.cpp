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

#include "autonomy/control/controller/neupan_controller/planner.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace autonomy {
namespace control {
namespace controller {
namespace neupan_controller {
namespace {

RobotModel MakeRobot(const NeuPanPlanner::Config& config) {
    if (config.vertices.cols() >= 3) {
        return RobotModel(config.kinematics, config.receding, config.step_time,
                          config.max_speed, config.max_acce, config.vertices);
    }
    return RobotModel::CreateRectangle(
        config.kinematics, config.receding, config.step_time, config.max_speed,
        config.max_acce, config.length, config.width, config.wheelbase);
}

}  // namespace

void NeuPanPlanner::Validate(const Config& config) {
    const NrmpParams& a = config.adjust;
    if (config.receding < 1 || config.step_time <= 0.0) {
        throw std::runtime_error("NeuPan: receding >= 1 and step_time > 0");
    }
    if (a.d_min < 0.0 || a.d_max < a.d_min) {
        throw std::runtime_error("NeuPan: invalid d_min/d_max");
    }
    if (config.collision_threshold <= 0.0) {
        throw std::runtime_error("NeuPan: collision_threshold must be > 0");
    }
}

NeuPanPlanner::NeuPanPlanner(const Config& config,
                             std::shared_ptr<const DistanceEncoder> encoder)
    : cfg_(config),
      robot_(MakeRobot(config)),
      ipath_(robot_, config.ref_speed, config.ipath),
      pan_(robot_, std::move(encoder), config.adjust, config.pan),
      ref_speed_(config.ref_speed),
      cur_vel_array_(Mat2X::Zero(2, config.receding)) {
    Validate(config);
}

Vec2 NeuPanPlanner::Forward(const Vec3& state, const Mat2X& points, Info* info,
                            const std::optional<Command>& command) {
    Info local;
    Info* out = info ? info : &local;
    *out = Info{};
    if (!ipath_.HasPath()) {
        return Vec2::Zero();
    }
    if (ipath_.CheckArrive(state)) {
        out->arrive = true;
        return Vec2::Zero();
    }
    const InitialPath::NomRef nom =
        ipath_.GenerateNomRefState(state, cur_vel_array_, ref_speed_);
    std::optional<Nrmp::Box> first;
    if (command) {
        first = FirstInputBox(*command, nom.nom_u(1, 0));
    }
    const Pan::Output pan_out =
        pan_.Forward(nom.nom_s, nom.nom_u, nom.ref_s, nom.ref_us, points, first);
    cur_vel_array_ = pan_out.opt_u;
    out->solved = pan_out.solved;
    out->solver_status = pan_out.solver_status;
    out->min_distance = pan_out.min_distance;
    out->opt_s = pan_out.opt_s;
    out->opt_u = pan_out.opt_u;
    out->ref_s = nom.ref_s;
    if (pan_out.min_distance < cfg_.collision_threshold) {
        out->stop = true;
        return Vec2::Zero();
    }
    const Vec2 u = pan_out.opt_u.col(0)
                       .cwiseMax(-robot_.max_speed)
                       .cwiseMin(robot_.max_speed);
    if (robot_.kinematics == Kinematics::kOmni) {
        return Vec2(u(0) * std::cos(u(1)), u(0) * std::sin(u(1)));
    }
    return u;
}

Nrmp::Box NeuPanPlanner::FirstInputBox(const Command& command,
                                       double nom_dir) const {
    const Vec2 b =
        robot_.acce_bound *
        std::clamp(command.elapsed / robot_.dt, 0.0, 1.0);
    const Vec2& a = command.action;
    if (robot_.kinematics != Kinematics::kOmni) {
        return {(a - b).cwiseMax(-robot_.max_speed),
                (a + b).cwiseMin(robot_.max_speed)};
    }
    const double speed = a.norm();
    if (speed <= b(0)) {
        return {Vec2(-b(0), -robot_.max_speed(1)),
                Vec2(b(0), robot_.max_speed(1))};
    }
    const double dir = nom_dir + WrapToPi(std::atan2(a.y(), a.x()) - nom_dir);
    const double half = std::asin(std::min(1.0, b(0) / speed));
    return {Vec2(speed - b(0), dir - half),
            Vec2(std::min(speed + b(0), robot_.max_speed(0)), dir + half)};
}

void NeuPanPlanner::Reset() {
    ipath_.Reset();
    pan_.Reset();
    cur_vel_array_.setZero();
}

}  // namespace neupan_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
