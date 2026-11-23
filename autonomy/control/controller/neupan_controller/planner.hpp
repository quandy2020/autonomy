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
 * @file planner.hpp
 * @brief Top-level NeuPAN planner (DUNE + NRMP + PAN).
 */

#include <limits>
#include <memory>
#include <optional>
#include <string>

#include "autonomy/control/controller/neupan_controller/distance_encoder.hpp"
#include "autonomy/control/controller/neupan_controller/initial_path.hpp"
#include "autonomy/control/controller/neupan_controller/pan.hpp"
#include "autonomy/control/controller/neupan_controller/robot_model.hpp"
#include "autonomy/control/controller/neupan_controller/types.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace neupan_controller {

class NeuPanPlanner
{
public:
    struct Config {
        int receding{10};
        double step_time{0.1};
        double ref_speed{0.5};
        double collision_threshold{0.1};
        Kinematics kinematics{Kinematics::kDiff};
        Vec2 max_speed{1.0, 1.0};
        Vec2 max_acce{1.0, 1.0};
        double length{0.8};
        double width{0.6};
        double wheelbase{0.0};
        Mat2X vertices;
        InitialPath::Options ipath;
        Pan::Options pan;
        NrmpParams adjust;
    };

    struct Info {
        bool arrive{false};
        bool stop{false};
        bool solved{false};
        int solver_status{-1};
        double min_distance{std::numeric_limits<double>::infinity()};
        Mat3X opt_s;
        Mat2X opt_u;
        Mat3X ref_s;
    };

    struct Command {
        Vec2 action{Vec2::Zero()};
        double elapsed{0.0};
    };

    NeuPanPlanner(const Config& config,
                  std::shared_ptr<const DistanceEncoder> encoder);

    static void Validate(const Config& config);

    Vec2 Forward(const Vec3& state, const Mat2X& points, Info* info,
                 const std::optional<Command>& command = std::nullopt);

    void Reset();
    void SetReferenceSpeed(double speed) {
        ref_speed_ = speed;
    }
    void SetInitialPath(std::vector<InitialPath::PathPoint> path) {
        ipath_.SetInitialPath(std::move(path));
    }
    void ReplaceInitialPath(std::vector<InitialPath::PathPoint> path,
                            const Vec3& state) {
        ipath_.ReplaceInitialPath(std::move(path), state);
    }
    const RobotModel& robot() const {
        return robot_;
    }

private:
    Nrmp::Box FirstInputBox(const Command& command, double nom_dir) const;

    Config cfg_;
    RobotModel robot_;
    InitialPath ipath_;
    Pan pan_;
    double ref_speed_{0.5};
    Mat2X cur_vel_array_;
};

}  // namespace neupan_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
