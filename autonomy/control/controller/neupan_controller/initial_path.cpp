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

#include "autonomy/control/controller/neupan_controller/initial_path.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace autonomy {
namespace control {
namespace controller {
namespace neupan_controller {

double WrapToPi(double rad) {
    while (rad > M_PI) {
        rad -= 2.0 * M_PI;
    }
    while (rad < -M_PI) {
        rad += 2.0 * M_PI;
    }
    return rad;
}

InitialPath::InitialPath(const RobotModel& robot, double ref_speed,
                         const Options& opts)
    : robot_(robot),
      T_(robot.T),
      dt_(robot.dt),
      ref_speed_(ref_speed),
      opts_(opts) {
    interval_ = opts.interval > 0.0 ? opts.interval : dt_ * ref_speed_;
}

std::vector<InitialPath::PathPoint> InitialPath::GenerateLineCurve(
    const std::vector<Vec3>& waypoints) const {
    std::vector<PathPoint> curve;
    if (waypoints.empty()) {
        return curve;
    }
    curve.emplace_back(waypoints[0].x(), waypoints[0].y(), waypoints[0].z(),
                       1.0);
    for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
        const Vec2 sp = waypoints[i].head<2>();
        const Vec2 ep = waypoints[i + 1].head<2>();
        const Vec2 diff = ep - sp;
        const double length = diff.norm();
        if (length < 1e-9) {
            continue;
        }
        const Vec2 dir = diff / length;
        const double theta = std::atan2(diff.y(), diff.x());
        double cur_len = 0.0;
        while (cur_len + interval_ < length) {
            cur_len += interval_;
            const Vec2 p = sp + cur_len * dir;
            curve.emplace_back(p.x(), p.y(), theta, 1.0);
        }
        curve.emplace_back(ep.x(), ep.y(), waypoints[i + 1].z(), 1.0);
    }
    for (size_t i = 0; i + 1 < curve.size(); ++i) {
        const double dx = curve[i + 1].x() - curve[i].x();
        const double dy = curve[i + 1].y() - curve[i].y();
        curve[i](2) = std::atan2(dy, dx);
    }
    if (curve.size() >= 2) {
        curve.back()(2) = curve[curve.size() - 2](2);
    }
    return curve;
}

void InitialPath::SplitPathWithGear() {
    curve_list_.clear();
    if (initial_path_.empty()) {
        return;
    }
    std::vector<PathPoint> current;
    double current_gear = initial_path_.front()(3);
    for (const auto& p : initial_path_) {
        if (p(3) != current_gear) {
            curve_list_.push_back(std::move(current));
            current.clear();
            current_gear = p(3);
        }
        current.push_back(p);
    }
    if (!current.empty()) {
        curve_list_.push_back(std::move(current));
    }
}

void InitialPath::SetPath(std::vector<PathPoint> path) {
    initial_path_ = std::move(path);
    SplitPathWithGear();
    curve_index_ = 0;
    point_index = 0;
}

void InitialPath::SetInitialPath(std::vector<PathPoint> path) {
    if (path.size() < 2) {
        throw std::invalid_argument("InitialPath needs at least 2 points");
    }
    std::vector<Vec3> waypoints;
    waypoints.reserve(path.size());
    for (const auto& p : path) {
        waypoints.emplace_back(p.head<3>());
    }
    SetPath(GenerateLineCurve(waypoints));
}

void InitialPath::ReplaceInitialPath(std::vector<PathPoint> path,
                                     const Vec3& state) {
    SetInitialPath(std::move(path));
    double best = std::numeric_limits<double>::infinity();
    for (size_t c = 0; c < curve_list_.size(); ++c) {
        const auto& curve = curve_list_[c];
        for (size_t i = 0; i < curve.size(); ++i) {
            const double dis = (state.head<2>() - curve[i].head<2>()).norm();
            if (dis < best) {
                best = dis;
                curve_index_ = static_cast<int>(c);
                point_index = static_cast<int>(i);
            }
        }
    }
    arrive_flag = false;
}

void InitialPath::Reset() {
    curve_index_ = 0;
    point_index = 0;
    arrive_flag = false;
}

double InitialPath::ClosestPoint(const Vec3& state, double threshold,
                                 int ind_range) {
    double min_dis = std::numeric_limits<double>::infinity();
    const auto& curve = CurCurve();
    const int start = std::max(point_index, 0);
    const int end = std::min(point_index + ind_range,
                             static_cast<int>(curve.size()));
    for (int i = start; i < end; ++i) {
        const double dis =
            (state.head<2>() - curve[static_cast<size_t>(i)].head<2>()).norm();
        if (dis < min_dis) {
            min_dis = dis;
            point_index = i;
            if (dis < threshold) {
                break;
            }
        }
    }
    return min_dis;
}

bool InitialPath::CheckCurveArrive(const Vec3& state) const {
    const auto& curve = CurCurve();
    const double arrive_distance =
        (state.head<2>() - curve.back().head<2>()).norm();
    return arrive_distance < opts_.arrive_threshold &&
           point_index >= static_cast<int>(curve.size()) -
                              opts_.arrive_index_threshold - 2;
}

bool InitialPath::CheckArrive(const Vec3& state) {
    if (!HasPath()) {
        return false;
    }
    if (arrive_flag) {
        return true;
    }
    ClosestPoint(state, opts_.close_threshold, opts_.ind_range);
    if (CheckCurveArrive(state)) {
        if (curve_index_ + 1 >= static_cast<int>(curve_list_.size())) {
            arrive_flag = true;
            return true;
        }
        ++curve_index_;
        point_index = 0;
    }
    return false;
}

Vec3 InitialPath::FindInteractionPoint(const Vec3& ref_state, int* ref_index,
                                       double length) const {
    const auto& curve = CurCurve();
    const Vec2 circle = ref_state.head<2>();
    while (true) {
        if (*ref_index > static_cast<int>(curve.size()) - 2) {
            Vec3 end = curve.back().head<3>();
            end(2) = WrapToPi(end(2));
            return end;
        }
        const PathPoint& cur = curve[static_cast<size_t>(*ref_index)];
        const PathPoint& next = curve[static_cast<size_t>(*ref_index + 1)];
        const Vec2 sp = cur.head<2>();
        const Vec2 ep = next.head<2>();
        const Vec2 d = ep - sp;
        bool found = false;
        Vec2 int_point = Vec2::Zero();
        if (d.norm() != 0.0) {
            const Vec2 f = sp - circle;
            const double a = d.dot(d);
            const double b = 2.0 * f.dot(d);
            const double c = f.dot(f) - length * length;
            const double disc = b * b - 4.0 * a * c;
            if (disc >= 0.0) {
                const double t2 = (-b + std::sqrt(disc)) / (2.0 * a);
                if (t2 >= 0.0 && t2 <= 1.0) {
                    int_point = sp + t2 * d;
                    found = true;
                }
            }
        }
        if (found) {
            const double diff = WrapToPi(next(2) - cur(2));
            const double theta = WrapToPi(cur(2) + diff / 2.0);
            return Vec3(int_point.x(), int_point.y(), theta);
        }
        ++(*ref_index);
    }
}

Vec3 InitialPath::MotionPredict(const Vec3& state, const Vec2& vel) const {
    if (robot_.kinematics == Kinematics::kOmni) {
        return state + dt_ * Vec3(vel(0) * std::cos(vel(1)),
                                  vel(0) * std::sin(vel(1)), 0.0);
    }
    const double phi = state(2);
    return state + dt_ * Vec3(vel(0) * std::cos(phi), vel(0) * std::sin(phi),
                              vel(1));
}

Vec2 InitialPath::SeedOmni(const Vec2& u, double path_dir, double ref_v,
                           double prev_dir) const {
    double v = u(0);
    double dir = u(1);
    if (v < 0.0) {
        v = -v;
        dir += M_PI;
    }
    const bool still = v < 0.01 * robot_.max_speed(0);
    if (still || std::cos(dir - path_dir) <= 0.0) {
        dir = path_dir;
    }
    if (still) {
        v = ref_v;
    }
    return Vec2(v, prev_dir + WrapToPi(dir - prev_dir));
}

InitialPath::NomRef InitialPath::GenerateNomRefState(
    const Vec3& state, const Mat2X& cur_vel_array, double ref_speed) const {
    const auto& curve = CurCurve();
    const int n = static_cast<int>(curve.size());
    Vec3 ref_state = curve[static_cast<size_t>(point_index)].head<3>();
    int ref_index = point_index;
    if (opts_.projected_reference) {
        double best = std::numeric_limits<double>::infinity();
        const int last = std::min(point_index + opts_.ind_range, n - 1);
        for (int i = std::max(point_index - 1, 0); i < last; ++i) {
            const Vec2 a = curve[static_cast<size_t>(i)].head<2>();
            const Vec2 d = curve[static_cast<size_t>(i + 1)].head<2>() - a;
            const double u =
                d.squaredNorm() > 0.0
                    ? std::clamp((state.head<2>() - a).dot(d) / d.squaredNorm(),
                                 0.0, 1.0)
                    : 0.0;
            const Vec2 p = a + u * d;
            const double dist = (state.head<2>() - p).norm();
            if (dist < best) {
                best = dist;
                ref_state << p, curve[static_cast<size_t>(i)](2);
                ref_index = i;
            }
        }
    }
    Vec3 pre_state = state;
    NomRef out;
    out.nom_s.resize(3, T_ + 1);
    out.ref_s.resize(3, T_ + 1);
    out.nom_u = cur_vel_array;
    out.ref_us.resize(T_);
    out.nom_s.col(0) = pre_state;
    out.ref_s.col(0) = ref_state;
    Vec gear = Vec::Constant(T_, curve[static_cast<size_t>(point_index)](3));
    const double ref_speed_forward = ref_speed * dt_;
    for (int t = 0; t < T_; ++t) {
        if (opts_.projected_reference) {
            ref_state =
                FindInteractionPoint(ref_state, &ref_index, ref_speed_forward);
            if (ref_index > n - 2) {
                gear(t) = 0.0;
            }
        } else if (ref_speed_forward >= interval_) {
            const int inc = static_cast<int>(ref_speed_forward / interval_);
            ref_index += inc;
            if (ref_index > n - 1) {
                ref_index = n - 1;
                gear(t) = 0.0;
            }
            ref_state = curve[static_cast<size_t>(ref_index)].head<3>();
        } else {
            ref_state =
                FindInteractionPoint(ref_state, &ref_index, ref_speed_forward);
            if (ref_index > n - 1) {
                gear(t) = 0.0;
            }
        }
        if (robot_.kinematics == Kinematics::kOmni) {
            out.nom_u.col(t) = SeedOmni(
                out.nom_u.col(t), ref_state(2), gear(t) * ref_speed,
                t > 0 ? out.nom_u(1, t - 1) : WrapToPi(cur_vel_array(1, 0)));
        }
        pre_state = MotionPredict(pre_state, out.nom_u.col(t));
        out.nom_s.col(t + 1) = pre_state;
        ref_state(2) = pre_state(2) + WrapToPi(ref_state(2) - pre_state(2));
        out.ref_s.col(t + 1) = ref_state;
    }
    out.ref_us = gear * ref_speed;
    return out;
}

}  // namespace neupan_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy
