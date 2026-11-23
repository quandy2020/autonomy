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
 * @file initial_path.hpp
 * @brief Line-style NeuPAN reference path and nominal rollout.
 */

#include <vector>

#include "autonomy/control/controller/neupan_controller/robot_model.hpp"
#include "autonomy/control/controller/neupan_controller/types.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace neupan_controller {

double WrapToPi(double rad);

class InitialPath
{
public:
    struct Options {
        std::vector<Vec3> waypoints;
        double interval{-1.0};
        double arrive_threshold{0.1};
        double close_threshold{0.1};
        int ind_range{10};
        int arrive_index_threshold{1};
        bool projected_reference{true};
    };

    using PathPoint = Eigen::Vector4d;

    InitialPath(const RobotModel& robot, double ref_speed, const Options& opts);

    bool CheckArrive(const Vec3& state);

    struct NomRef {
        Mat3X nom_s;
        Mat2X nom_u;
        Mat3X ref_s;
        Vec ref_us;
    };
    NomRef GenerateNomRefState(const Vec3& state, const Mat2X& cur_vel_array,
                               double ref_speed) const;

    void SetInitialPath(std::vector<PathPoint> path);
    void ReplaceInitialPath(std::vector<PathPoint> path, const Vec3& state);
    void Reset();
    bool HasPath() const {
        return !curve_list_.empty();
    }

    int point_index{0};
    bool arrive_flag{false};

private:
    std::vector<PathPoint> GenerateLineCurve(
        const std::vector<Vec3>& waypoints) const;
    void SetPath(std::vector<PathPoint> path);
    void SplitPathWithGear();
    double ClosestPoint(const Vec3& state, double threshold, int ind_range);
    bool CheckCurveArrive(const Vec3& state) const;
    Vec3 FindInteractionPoint(const Vec3& ref_state, int* ref_index,
                              double length) const;
    Vec3 MotionPredict(const Vec3& state, const Vec2& vel) const;
    Vec2 SeedOmni(const Vec2& u, double path_dir, double ref_v,
                  double prev_dir) const;
    const std::vector<PathPoint>& CurCurve() const {
        return curve_list_[static_cast<size_t>(curve_index_)];
    }

    RobotModel robot_;
    int T_{0};
    double dt_{0.0};
    double ref_speed_{0.0};
    Options opts_;
    double interval_{0.0};
    std::vector<PathPoint> initial_path_;
    std::vector<std::vector<PathPoint>> curve_list_;
    int curve_index_{0};
};

}  // namespace neupan_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
