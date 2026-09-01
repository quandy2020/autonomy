/*
 * Copyright 2026 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "autonomy/task/teleop/path_selector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <utility>

#include "autonomy/map/costmap_2d/cost_values.hpp"

namespace autonomy::task::teleop {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kStoppedSpeedEpsilon = 1e-6;
constexpr int kDefaultNumRotDirs = 36;
constexpr int kForwardRotDir = 18;

double WrapDeg(double deg) {
    deg = std::fmod(deg + 180.0, 360.0);
    if (deg < 0.0) {
        deg += 360.0;
    }
    return deg - 180.0;
}

automsgs::msgs::geometry_msgs::PoseStamped MakePose(double x, double y,
                                                    double yaw) {
    automsgs::msgs::geometry_msgs::PoseStamped pose{};
    pose.mutable_pose()->mutable_position()->set_x(x);
    pose.mutable_pose()->mutable_position()->set_y(y);
    pose.mutable_pose()->mutable_orientation()->set_z(std::sin(yaw * 0.5));
    pose.mutable_pose()->mutable_orientation()->set_w(std::cos(yaw * 0.5));
    return pose;
}

/**
 * @class CubicSpline1D
 * @brief Natural cubic spline y(x) for monotone-increasing x knots
 */
class CubicSpline1D
{
public:
    void SetPoints(std::vector<double> x, std::vector<double> y)
    {
        x_ = std::move(x);
        y_ = std::move(y);
        const std::size_t n = x_.size();
        if (n < 2) {
            valid_ = false;
            return;
        }
        valid_ = true;
        b_.assign(n - 1, 0.0);
        c_.assign(n, 0.0);
        d_.assign(n - 1, 0.0);

        std::vector<double> h(n - 1);
        for (std::size_t i = 0; i < n - 1; ++i) {
            h[i] = x_[i + 1] - x_[i];
            if (h[i] <= 0.0) {
                valid_ = false;
                return;
            }
        }

        std::vector<double> alpha(n - 1, 0.0);
        for (std::size_t i = 1; i < n - 1; ++i) {
            alpha[i] = (3.0 / h[i]) * (y_[i + 1] - y_[i]) -
                       (3.0 / h[i - 1]) * (y_[i] - y_[i - 1]);
        }

        std::vector<double> l(n, 0.0);
        std::vector<double> mu(n, 0.0);
        std::vector<double> z(n, 0.0);
        l[0] = 1.0;
        for (std::size_t i = 1; i < n - 1; ++i) {
            l[i] = 2.0 * (x_[i + 1] - x_[i - 1]) - h[i - 1] * mu[i - 1];
            if (std::abs(l[i]) < 1e-12) {
                valid_ = false;
                return;
            }
            mu[i] = h[i] / l[i];
            z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
        }
        l[n - 1] = 1.0;
        z[n - 1] = 0.0;
        c_[n - 1] = 0.0;

        for (int j = static_cast<int>(n) - 2; j >= 0; --j) {
            c_[j] = z[j] - mu[j] * c_[j + 1];
            b_[j] = (y_[j + 1] - y_[j]) / h[j] -
                    h[j] * (c_[j + 1] + 2.0 * c_[j]) / 3.0;
            d_[j] = (c_[j + 1] - c_[j]) / (3.0 * h[j]);
        }
    }

    double Evaluate(double x) const
    {
        if (!valid_ || x_.empty()) {
            return 0.0;
        }
        if (x <= x_.front()) {
            return y_.front();
        }
        if (x >= x_.back()) {
            return y_.back();
        }
        const auto it =
            std::upper_bound(x_.begin(), x_.end(), x) - x_.begin() - 1;
        const std::size_t i = static_cast<std::size_t>(it);
        const double dx = x - x_[i];
        return y_[i] + b_[i] * dx + c_[i] * dx * dx + d_[i] * dx * dx * dx;
    }

private:
    std::vector<double> x_;
    std::vector<double> y_;
    std::vector<double> b_;
    std::vector<double> c_;
    std::vector<double> d_;
    bool valid_{false};
};

automsgs::msgs::nav_msgs::Path SamplePolarSplineFromKnots(
    const std::vector<double>& knots_r, const std::vector<double>& knots_h,
    double sample_ds, double max_r) {
    automsgs::msgs::nav_msgs::Path path;
    if (knots_r.size() < 2 || knots_h.size() != knots_r.size() ||
        sample_ds <= 0.0 || max_r <= 0.0) {
        return path;
    }

    CubicSpline1D spline;
    spline.SetPoints(knots_r, knots_h);

    const int num_steps = static_cast<int>(std::ceil(max_r / sample_ds));
    for (int step = 0; step <= num_steps; ++step) {
        const double r = std::min(step * sample_ds, max_r);
        const double heading_deg = spline.Evaluate(r);
        const double heading_rad = heading_deg * kPi / 180.0;
        const double x = r * std::cos(heading_rad);
        const double y = r * std::sin(heading_rad);
        *path.add_poses() = MakePose(x, y, heading_rad);
    }
    return path;
}

automsgs::msgs::nav_msgs::Path SamplePolarSplinePath(
    const std::vector<std::pair<double, double>>& waypoints_r_heading_deg,
    double sample_ds, double max_r) {
    std::vector<double> knots_r;
    std::vector<double> knots_h;
    knots_r.reserve(waypoints_r_heading_deg.size());
    knots_h.reserve(waypoints_r_heading_deg.size());
    for (const auto& [r, h] : waypoints_r_heading_deg) {
        knots_r.push_back(r);
        knots_h.push_back(h);
    }
    return SamplePolarSplineFromKnots(knots_r, knots_h, sample_ds, max_r);
}

void BuildStartPathKnots(double dis, double shift1, double knot_ds,
                         std::vector<double>* knots_r,
                         std::vector<double>* knots_h) {
    if (knots_r == nullptr || knots_h == nullptr || dis <= 0.0 ||
        knot_ds <= 0.0) {
        return;
    }
    CubicSpline1D spline;
    spline.SetPoints(std::vector<double>{0.0, dis},
                     std::vector<double>{0.0, shift1});
    knots_r->clear();
    knots_h->clear();
    for (double r = 0.0; r <= dis + 1e-9; r += knot_ds) {
        knots_r->push_back(r);
        knots_h->push_back(spline.Evaluate(r));
    }
    if (knots_r->empty() || knots_r->back() < dis - 1e-9) {
        knots_r->push_back(dis);
        knots_h->push_back(shift1);
    }
}

automsgs::msgs::nav_msgs::Path TrimPathToArcLength(
    const automsgs::msgs::nav_msgs::Path& path, double max_length) {
    automsgs::msgs::nav_msgs::Path trimmed;
    if (path.poses_size() == 0 || max_length <= 0.0) {
        return trimmed;
    }
    *trimmed.mutable_header() = path.header();
    *trimmed.add_poses() = path.poses(0);
    if (path.poses_size() == 1) {
        return trimmed;
    }

    double accumulated = 0.0;
    for (int i = 1; i < path.poses_size(); ++i) {
        const auto& prev = path.poses(i - 1).pose().position();
        const auto& curr = path.poses(i).pose().position();
        const double dx = curr.x() - prev.x();
        const double dy = curr.y() - prev.y();
        const double seg = std::hypot(dx, dy);
        if (seg <= 1e-9) {
            continue;
        }
        if (accumulated + seg >= max_length) {
            const double ratio = (max_length - accumulated) / seg;
            auto pose = path.poses(i);
            pose.mutable_pose()->mutable_position()->set_x(prev.x() + ratio * dx);
            pose.mutable_pose()->mutable_position()->set_y(prev.y() + ratio * dy);
            *trimmed.add_poses() = pose;
            break;
        }
        accumulated += seg;
        *trimmed.add_poses() = path.poses(i);
    }
    return trimmed;
}

double RotDirToDeg(int rot_dir) {
    return 10.0 * static_cast<double>(rot_dir) - 180.0;
}

double RotDirWeight(int rot_dir) {
    if (rot_dir < 18) {
        return std::abs(std::abs(rot_dir - 9) + 1);
    }
    return std::abs(std::abs(rot_dir - 27) + 1);
}

bool JoyRotGate(double joy_dir_deg, int rot_dir, double dir_threshold,
                         bool dir_to_vehicle) {
    const double rot_deg = RotDirToDeg(rot_dir);
    if (!dir_to_vehicle) {
        double ang_diff = std::abs(joy_dir_deg - rot_deg);
        if (ang_diff > 180.0) {
            ang_diff = 360.0 - ang_diff;
        }
        return ang_diff <= dir_threshold;
    }
    if (std::abs(joy_dir_deg) <= 90.0) {
        return std::abs(rot_deg) <= dir_threshold;
    }
    const double rot_raw = 10.0 * static_cast<double>(rot_dir);
    return !(rot_raw > dir_threshold && 360.0 - rot_raw > dir_threshold);
}

RotObstacleAngles RotObstacleScan(const PathObstacleGrid& grid,
                                           double path_scale,
                                           double vehicle_length,
                                           double vehicle_width) {
    RotObstacleAngles angles;
    if (path_scale <= 1e-6) {
        path_scale = 1.0;
    }
    const double half_l = vehicle_length / (2.0 * path_scale);
    const double half_w = vehicle_width / (2.0 * path_scale);
    const double diameter =
        std::hypot(vehicle_length / 2.0, vehicle_width / 2.0);
    const double ang_offset =
        std::atan2(vehicle_width, vehicle_length) * 180.0 / kPi;
    const double scan_r = diameter / path_scale;
    const double step = 0.05;

    for (double wx = -scan_r; wx <= scan_r + 1e-6; wx += step) {
        for (double wy = -scan_r; wy <= scan_r + 1e-6; wy += step) {
            const double x = wx / path_scale;
            const double y = wy / path_scale;
            const double dis = std::hypot(x, y);
            if (dis >= scan_r || (std::abs(x) <= half_l && std::abs(y) <= half_w)) {
                continue;
            }
            if (!IsPathBlocked(grid, wx, wy)) {
                continue;
            }
            const double ang_obs = std::atan2(y, x) * 180.0 / kPi;
            if (ang_obs > 0.0) {
                angles.min_ccw =
                    std::min(angles.min_ccw, ang_obs - ang_offset);
                angles.min_cw = std::max(angles.min_cw,
                                           ang_obs + ang_offset - 180.0);
            } else {
                angles.min_cw =
                    std::min(angles.min_cw, ang_obs + ang_offset);
                angles.min_ccw = std::max(
                    angles.min_ccw, 180.0 + ang_obs - ang_offset);
            }
        }
    }
    if (angles.min_cw > 0.0) {
        angles.min_cw = 0.0;
    }
    if (angles.min_ccw < 0.0) {
        angles.min_ccw = 0.0;
    }
    return angles;
}

bool RotDirClear(int rot_dir, const RotObstacleAngles& angles,
                              bool check_rot_obstacle, bool two_way_drive) {
    if (!check_rot_obstacle) {
        return true;
    }
    const double rot_deg = RotDirToDeg(rot_dir);
    double rot_deg_alt = 10.0 * static_cast<double>(rot_dir);
    if (rot_deg_alt > 180.0) {
        rot_deg_alt -= 360.0;
    }
    if ((rot_deg > angles.min_cw && rot_deg < angles.min_ccw) ||
        (rot_deg_alt > angles.min_cw && rot_deg_alt < angles.min_ccw &&
         two_way_drive)) {
        return true;
    }
    return false;
}

double AngDiffDeg(double a_deg, double b_deg) {
    double diff = std::abs(a_deg - b_deg);
    if (diff > 180.0) {
        diff = 360.0 - diff;
    }
    return diff;
}

int BestRotDir(double joy_dir_deg, const RotObstacleAngles& angles,
                       const PathLibraryOptions& options) {
    const int num_rot_dirs =
        options.num_rot_dirs > 0 ? options.num_rot_dirs : 36;
    int best_rot_dir = -1;
    double best_diff = 360.0;
    for (int rot_dir = 0; rot_dir < num_rot_dirs; ++rot_dir) {
        if (!JoyRotGate(joy_dir_deg, rot_dir, options.dir_threshold_deg,
                                 options.dir_to_vehicle)) {
            continue;
        }
        if (!RotDirClear(rot_dir, angles, true,
                                      options.two_way_drive)) {
            continue;
        }
        const double diff =
            AngDiffDeg(joy_dir_deg, RotDirToDeg(rot_dir));
        if (diff < best_diff) {
            best_diff = diff;
            best_rot_dir = rot_dir;
        }
    }
    return best_rot_dir;
}

void FillPreviewScales(PathSelectionResult* result,
                               double joy_speed_norm,
                               const PathLibraryOptions& options) {
    if (result == nullptr || result->best_path.has_value()) {
        return;
    }
    const double def_scale =
        options.def_path_scale > 0.0 ? options.def_path_scale : 1.25;
    const double max_range = options.path_range > 0.0
                                 ? options.path_range
                                 : 3.0 * options.segment_length;
    const double speed = std::max(joy_speed_norm, 0.05);
    result->best_path_scale =
        options.path_scale_by_speed ? def_scale * speed : def_scale;
    result->best_path_scale =
        std::max(result->best_path_scale, std::max(0.1, options.min_path_scale));
    result->active_range =
        options.path_range_by_speed
            ? std::max(options.min_path_range, max_range * speed)
            : max_range;
    result->scaled_range = result->active_range / result->best_path_scale;
}

bool JoyRotGate(double joy_dir_deg, int rot_dir, double dir_threshold) {
    return JoyRotGate(joy_dir_deg, rot_dir, dir_threshold, false);
}

automsgs::msgs::nav_msgs::Path RotateAndScalePath(
    const automsgs::msgs::nav_msgs::Path& path, double rot_deg, double scale) {
    automsgs::msgs::nav_msgs::Path out = path;
    if (out.poses_size() == 0) {
        return out;
    }
    const double rot_rad = rot_deg * kPi / 180.0;
    const double c = std::cos(rot_rad);
    const double s = std::sin(rot_rad);
    for (auto& pose_stamped : *out.mutable_poses()) {
        const double x = pose_stamped.pose().position().x();
        const double y = pose_stamped.pose().position().y();
        pose_stamped.mutable_pose()->mutable_position()->set_x(
            scale * (c * x - s * y));
        pose_stamped.mutable_pose()->mutable_position()->set_y(
            scale * (s * x + c * y));
        const double yaw =
            2.0 * std::atan2(pose_stamped.pose().orientation().z(),
                             pose_stamped.pose().orientation().w());
        const double new_yaw = yaw + rot_rad;
        pose_stamped.mutable_pose()->mutable_orientation()->set_z(
            std::sin(new_yaw * 0.5));
        pose_stamped.mutable_pose()->mutable_orientation()->set_w(
            std::cos(new_yaw * 0.5));
    }
    return out;
}

double PathIntentScore(double end_dir_deg, double joy_dir_deg, int rot_dir,
                             double dir_weight) {
    const double rot_deg = RotDirToDeg(rot_dir);
    double dir_diff = std::abs(joy_dir_deg - end_dir_deg - rot_deg);
    while (dir_diff > 360.0) {
        dir_diff -= 360.0;
    }
    if (dir_diff > 180.0) {
        dir_diff = 360.0 - dir_diff;
    }
    const double weighted = dir_weight * dir_diff;
    if (weighted <= 0.0) {
        const double rot_w = RotDirWeight(rot_dir);
        return rot_w * rot_w * rot_w * rot_w;
    }
    const double base = 1.0 - std::sqrt(std::sqrt(weighted));
    if (base <= 0.0) {
        return 0.0;
    }
    const double rot_w = RotDirWeight(rot_dir);
    return base * rot_w * rot_w * rot_w * rot_w;
}

double EndpointBearing(const automsgs::msgs::nav_msgs::Path& path,
                              double origin_x = 0.0, double origin_y = 0.0) {
    if (path.poses_size() == 0) {
        return 0.0;
    }
    const auto& end = path.poses(path.poses_size() - 1).pose().position();
    return std::atan2(end.y() - origin_y, end.x() - origin_x) * 180.0 / kPi;
}

double MaxAbsBearing(const automsgs::msgs::nav_msgs::Path& path,
                            double origin_x, double origin_y) {
    double max_abs = 0.0;
    for (const auto& pose_stamped : path.poses()) {
        const double x = pose_stamped.pose().position().x() - origin_x;
        const double y = pose_stamped.pose().position().y() - origin_y;
        if (x * x + y * y < 1e-8) {
            continue;
        }
        max_abs = std::max(
            max_abs, std::abs(std::atan2(y, x) * 180.0 / kPi));
    }
    return max_abs;
}

double CompositePathScore(
    double intent_score, const PathObstacleGrid& grid,
    const automsgs::msgs::nav_msgs::Path& world_path,
    const PathLibraryOptions& options, double active_range,
    const PathSelectContext& context, int rot_dir, int group_id,
    const PathTemporalState* last_selection) {
    if (intent_score <= 0.0) {
        return 0.0;
    }
    double score = intent_score;
    if (options.clearance_weight > 0.0) {
        const double penalty = PathClearancePenalty(grid, world_path);
        score *= (1.0 - options.clearance_weight * penalty);
    }
    if (options.smoothness_weight > 0.0) {
        const double penalty = PathSmoothnessPenalty(world_path);
        score *= (1.0 - options.smoothness_weight * penalty);
    }
    if (options.efficiency_weight > 0.0) {
        const double penalty = PathEfficiencyPenalty(world_path, active_range);
        score *= (1.0 - options.efficiency_weight * penalty);
    }
    if (options.velocity_continuity_weight > 0.0) {
        const double penalty = VelContinuityPenalty(
            world_path, context.robot_vx, context.robot_wz);
        score *= (1.0 - options.velocity_continuity_weight * penalty);
    }
    if (options.temporal_weight > 0.0 && last_selection != nullptr &&
        last_selection->valid) {
        double bonus = 0.0;
        if (rot_dir == last_selection->rot_dir) {
            bonus += 1.0;
        }
        if (group_id == last_selection->group_id) {
            bonus += 0.5;
        }
        if (bonus > 0.0) {
            score *= (1.0 + options.temporal_weight * bonus);
        }
    }
    if (options.traversability_weight > 0.0) {
        const double traversability =
            PathMeanTraversability(grid, world_path);
        score *= (1.0 - options.traversability_weight) +
                  options.traversability_weight * traversability;
    }
    return std::max(0.0, score);
}

void FillFreeIndices(
    PathSelectionResult* result, const PathObstacleGrid& grid,
    const std::vector<PathCandidate>& candidates, double path_scale,
    double scaled_range, int lethal_threshold,
    const PathLibraryOptions& options) {
    if (result == nullptr) {
        return;
    }
    result->free_path_indices.clear();
    const double rot_deg =
        options.free_paths_in_base_link ? 0.0 : result->best_rot_deg;
    for (std::size_t index = 0; index < candidates.size(); ++index) {
        automsgs::msgs::nav_msgs::Path library_path =
            TrimPathToArcLength(candidates[index].path, scaled_range);
        if (library_path.poses_size() < 2) {
            continue;
        }
        const automsgs::msgs::nav_msgs::Path world_path =
            RotateAndScalePath(library_path, rot_deg, path_scale);
        if (!IntentPathSelector::InRgbdFov(candidates[index],
                                                   world_path, options)) {
            continue;
        }
        if (IntentPathSelector::CountLethalHits(grid, world_path) >=
            lethal_threshold) {
            continue;
        }
        result->free_path_indices.push_back(static_cast<int>(index));
    }
}

}  // namespace

/**
 * @brief Generate path library from options
 */
void IntentPathSelector::GenerateLibrary(const PathLibraryOptions& options) {
    library_options_ = options;
    if (options.use_polar_spline) {
        BuildSplineLib(options);
    } else {
        GenerateArcLibrary(options.num_dirs, options.num_lengths,
                           options.max_range, options.ds);
    }
}

/**
 * @brief Generate legacy arc library
 */
void IntentPathSelector::GenerateArcLib(int num_dirs, int num_lengths,
                                                double max_range, double ds) {
    PathLibraryOptions options;
    options.use_polar_spline = false;
    options.num_dirs = num_dirs;
    options.num_lengths = num_lengths;
    options.max_range = max_range;
    options.ds = ds;
    GenerateLibrary(options);
}

/**
 * @brief Build CMU polar spline candidate set
 */
void IntentPathSelector::BuildSplineLib(
    const PathLibraryOptions& options) {
    candidates_.clear();
    group_start_paths_.clear();

    const double dis = options.segment_length;
    const double angle = options.max_heading_deg;
    const double delta_angle = angle / 3.0;
    const double scale = options.hierarchy_scale;
    // CMU path_generator.m: paths end at 3 * dis.
    const double path_end =
        options.path_range > 0.0 ? options.path_range : 3.0 * dis;
    const double ds = options.sample_ds;
    const double knot_ds = options.library_knot_ds > 0.0
                               ? options.library_knot_ds
                               : 0.01;

    if (dis <= 0.0 || path_end <= 0.0 || ds <= 0.0) {
        return;
    }

    int path_id = 0;
    int group_id = 0;
    for (double shift1 = -angle; shift1 <= angle + 1e-6; shift1 += delta_angle) {
        std::vector<double> start_r;
        std::vector<double> start_h;
        BuildStartPathKnots(dis, shift1, knot_ds, &start_r, &start_h);
        group_start_paths_.push_back(
            SamplePolarSplineFromKnots(start_r, start_h, ds, dis));

        for (double shift2 = -angle * scale + shift1;
             shift2 <= angle * scale + shift1 + 1e-6;
             shift2 += delta_angle * scale) {
            for (double shift3 = -angle * scale * scale + shift2;
                 shift3 <= angle * scale * scale + shift2 + 1e-6;
                 shift3 += delta_angle * scale * scale) {
                std::vector<double> knots_r = start_r;
                std::vector<double> knots_h = start_h;
                knots_r.push_back(2.0 * dis);
                knots_h.push_back(shift2);
                knots_r.push_back(path_end - 1e-3);
                knots_h.push_back(shift3);
                knots_r.push_back(path_end);
                knots_h.push_back(shift3);

                PathCandidate candidate;
                candidate.id = path_id++;
                candidate.group_id = group_id;
                candidate.path =
                    SamplePolarSplineFromKnots(knots_r, knots_h, ds, path_end);
                if (candidate.path.poses_size() < 2) {
                    continue;
                }
                const auto& end_pose =
                    candidate.path.poses(candidate.path.poses_size() - 1);
                candidate.end_dir_deg = EndDirectionDeg(
                    end_pose.pose().position().x(),
                    end_pose.pose().position().y());
                candidates_.push_back(std::move(candidate));
            }
        }
        ++group_id;
    }
}

/**
 * @brief Build legacy arc candidate set
 */
void IntentPathSelector::GenerateArcLibrary(int num_dirs, int num_lengths,
                                            double max_range, double ds) {
    candidates_.clear();
    if (num_dirs <= 0 || num_lengths <= 0 || max_range <= 0.0 || ds <= 0.0) {
        return;
    }

    int path_id = 0;
    candidates_.reserve(static_cast<std::size_t>(num_dirs * num_lengths));
    for (int dir_index = 0; dir_index < num_dirs; ++dir_index) {
        const double end_dir_deg =
            num_dirs == 1
                ? 0.0
                : -90.0 + 180.0 * dir_index / static_cast<double>(num_dirs - 1);
        const double end_dir_rad = end_dir_deg * kPi / 180.0;

        for (int length_index = 1; length_index <= num_lengths;
             ++length_index) {
            const double length =
                max_range * length_index / static_cast<double>(num_lengths);
            const double curvature = end_dir_rad / length;

            PathCandidate candidate;
            candidate.id = path_id++;
            candidate.group_id = dir_index;
            candidate.end_dir_deg = end_dir_deg;
            const int num_steps = static_cast<int>(std::ceil(length / ds));
            candidate.path.mutable_poses()->Reserve(num_steps + 1);
            *candidate.path.add_poses() = MakePose(0.0, 0.0, 0.0);

            for (int step = 1; step <= num_steps; ++step) {
                const double arc_length = std::min(step * ds, length);
                const double yaw = curvature * arc_length;
                if (std::abs(curvature) < 1e-12) {
                    *candidate.path.add_poses() =
                        MakePose(arc_length, 0.0, yaw);
                } else {
                    *candidate.path.add_poses() = MakePose(
                        std::sin(yaw) / curvature,
                        (1.0 - std::cos(yaw)) / curvature, yaw);
                }
            }
            candidates_.push_back(std::move(candidate));
        }
    }
}

/**
 * @brief Select best path for joystick intent and costmap
 */
PathSelectionResult IntentPathSelector::Select(const PathObstacleGrid& grid,
                                               double joy_dir_deg,
                                               double joy_speed,
                                               const PathSelectContext& context) const {
    PathSelectionResult result;
    if (std::abs(joy_speed) <= kStoppedSpeedEpsilon || candidates_.empty()) {
        return result;
    }

    const double max_range = [&]() {
        double range = library_options_.path_range > 0.0
                             ? library_options_.path_range
                             : library_options_.max_range;
        if (library_options_.sensor_range > 0.0) {
            range = std::min(range, library_options_.sensor_range);
        }
        return range;
    }();
    const double min_range = std::max(0.1, library_options_.min_path_range);
    const double range_step = std::max(0.1, library_options_.path_range_step);
    const bool use_groups = library_options_.use_group_selection;
    const int lethal_threshold = library_options_.point_per_path_thr > 0
                                     ? library_options_.point_per_path_thr
                                     : point_per_path_thre_;
    const double dir_weight = library_options_.dir_weight > 0.0
                                  ? library_options_.dir_weight
                                  : dir_weight_;
    const double dir_threshold = library_options_.dir_threshold_deg;
    const double max_linear = library_options_.max_linear_speed > 1e-6
                                  ? library_options_.max_linear_speed
                                  : 0.5;
    const bool use_rot_search = library_options_.use_rot_dir_search;
    const int num_rot_dirs =
        library_options_.num_rot_dirs > 0 ? library_options_.num_rot_dirs
                                          : kDefaultNumRotDirs;
    const int num_groups = group_start_paths_.empty()
                               ? 1
                               : static_cast<int>(group_start_paths_.size());

    const bool dir_to_vehicle = library_options_.dir_to_vehicle;
    const bool check_rot_obstacle = library_options_.check_rot_obstacle;
    const bool two_way_drive = library_options_.two_way_drive;

    const double speed_norm = joy_speed > 1.0 + 1e-6
                                  ? std::clamp(std::abs(joy_speed) / max_linear,
                                                0.0, 1.0)
                                  : std::clamp(std::abs(joy_speed), 0.0, 1.0);
    const double speed_scaled_max =
        library_options_.path_range_by_speed
            ? std::max(min_range, max_range * std::max(speed_norm, 0.05))
            : max_range;

    const double def_path_scale = library_options_.def_path_scale > 0.0
                                      ? library_options_.def_path_scale
                                      : 1.25;
    double path_scale = def_path_scale;
    if (library_options_.path_scale_by_speed) {
        path_scale *= std::max(speed_norm, 0.05);
    }
    const double min_path_scale =
        std::max(0.1, library_options_.min_path_scale);
    if (path_scale < min_path_scale) {
        path_scale = min_path_scale;
    }
    const double path_scale_step =
        std::max(0.05, library_options_.path_scale_step);

    result.scores.assign(candidates_.size(), 0.0);

    while (path_scale >= min_path_scale - 1e-6) {
        const RotObstacleAngles rot_obs_angles =
            check_rot_obstacle
                ? RotObstacleScan(
                      grid, path_scale,
                      library_options_.rot_obstacle_vehicle_length,
                      library_options_.rot_obstacle_vehicle_width)
                : RotObstacleAngles{};
        const double range_at_scale = std::max(
            min_range, speed_scaled_max * path_scale / def_path_scale);
        for (double active_range = range_at_scale;
             active_range >= min_range - 1e-6; active_range -= range_step) {
            result.feasible_indices.clear();
            std::fill(result.scores.begin(), result.scores.end(), 0.0);
            result.best_score = -std::numeric_limits<double>::infinity();
            result.best_index = -1;
            result.best_group_id = -1;

            std::vector<double> group_rot_scores(
                static_cast<std::size_t>(num_rot_dirs * num_groups), 0.0);
            std::vector<int> group_rot_counts(
                static_cast<std::size_t>(num_rot_dirs * num_groups), 0);
            std::vector<int> best_index_in_group_rot(
                static_cast<std::size_t>(num_rot_dirs * num_groups), -1);
            std::vector<double> best_score_in_group_rot(
                static_cast<std::size_t>(num_rot_dirs * num_groups), 0.0);

            const int rot_begin = use_rot_search ? 0 : kForwardRotDir;
            const int rot_end = use_rot_search ? num_rot_dirs : kForwardRotDir + 1;

            for (int rot_dir = rot_begin; rot_dir < rot_end; ++rot_dir) {
                if (!JoyRotGate(joy_dir_deg, rot_dir, dir_threshold,
                                         dir_to_vehicle)) {
                    continue;
                }
                if (!RotDirClear(rot_dir, rot_obs_angles,
                                              check_rot_obstacle,
                                              two_way_drive)) {
                    continue;
                }
                const double rot_deg = RotDirToDeg(rot_dir);
                const double scaled_range = active_range / path_scale;

                for (std::size_t index = 0; index < candidates_.size();
                     ++index) {
                    const auto& candidate = candidates_[index];
                    automsgs::msgs::nav_msgs::Path library_path =
                        TrimPathToArcLength(candidate.path, scaled_range);
                    if (library_path.poses_size() < 2) {
                        continue;
                    }

                    const automsgs::msgs::nav_msgs::Path world_path =
                        RotateAndScalePath(library_path, rot_deg, path_scale);
                    const int lethal_hits = CountLethalHits(grid, world_path);
                    if (lethal_hits >= lethal_threshold) {
                        continue;
                    }

                    const double intent_score = PathIntentScore(
                        candidate.end_dir_deg, joy_dir_deg, rot_dir,
                        dir_weight);
                    const double score = CompositePathScore(
                        intent_score, grid, world_path, library_options_,
                        active_range, context, rot_dir, candidate.group_id,
                        temporal_state_.valid ? &temporal_state_ : nullptr);
                    if (score <= 0.0) {
                        continue;
                    }

                    result.scores[index] =
                        std::max(result.scores[index], score);
                    if (!library_options_.plot_path_set) {
                        if (std::find(result.feasible_indices.begin(),
                                      result.feasible_indices.end(),
                                      static_cast<int>(index)) ==
                            result.feasible_indices.end()) {
                            result.feasible_indices.push_back(
                                static_cast<int>(index));
                        }
                    }

                    if (!use_groups) {
                        if (score > result.best_score) {
                            result.best_score = score;
                            result.best_index = static_cast<int>(index);
                            result.best_group_id = candidate.group_id;
                            result.best_rot_dir = rot_dir;
                            result.best_rot_deg = rot_deg;
                            result.best_path_scale = path_scale;
                        }
                        continue;
                    }

                    const int group_rot_key =
                        rot_dir * num_groups + candidate.group_id;
                    if (group_rot_key < 0 ||
                        group_rot_key >= num_rot_dirs * num_groups) {
                        continue;
                    }
                    const std::size_t key =
                        static_cast<std::size_t>(group_rot_key);
                    group_rot_scores[key] += score;
                    ++group_rot_counts[key];
                    if (score > best_score_in_group_rot[key]) {
                        best_score_in_group_rot[key] = score;
                        best_index_in_group_rot[key] = static_cast<int>(index);
                    }
                }
            }

            int chosen_rot_dir = result.best_rot_dir;
            int chosen_group = result.best_group_id;
            int chosen_index = result.best_index;
            double chosen_score = result.best_score;

            if (use_groups) {
                chosen_rot_dir = -1;
                chosen_group = -1;
                chosen_index = -1;
                chosen_score = -std::numeric_limits<double>::infinity();
                for (int rot_dir = rot_begin; rot_dir < rot_end; ++rot_dir) {
                    if (!RotDirClear(rot_dir, rot_obs_angles,
                                                  check_rot_obstacle,
                                                  two_way_drive)) {
                        continue;
                    }
                    for (int group_id = 0; group_id < num_groups; ++group_id) {
                        const std::size_t key = static_cast<std::size_t>(
                            rot_dir * num_groups + group_id);
                        const double aggregate = group_rot_scores[key];
                        if (aggregate <= 0.0) {
                            continue;
                        }
                        const double normalized =
                            library_options_.normalize_group_scores
                                ? aggregate /
                                      static_cast<double>(std::max(
                                          1, group_rot_counts[key]))
                                : aggregate;
                        if (normalized > chosen_score) {
                            chosen_score = normalized;
                            chosen_rot_dir = rot_dir;
                            chosen_group = group_id;
                            chosen_index = best_index_in_group_rot[key];
                        }
                    }
                }
            }

            if (chosen_index >= 0 &&
                chosen_index < static_cast<int>(candidates_.size()) &&
                chosen_group >= 0) {
                const double rot_deg = RotDirToDeg(chosen_rot_dir);
                const double scaled_range = active_range / path_scale;
                const automsgs::msgs::nav_msgs::Path& source_path =
                    candidates_[static_cast<std::size_t>(chosen_index)].path;
                automsgs::msgs::nav_msgs::Path follow_path =
                    BuildFollowPath(source_path, scaled_range, rot_deg,
                                            path_scale);
                if (follow_path.poses_size() >= 2) {
                    result.best_index = chosen_index;
                    result.best_group_id = chosen_group;
                    result.best_rot_dir = chosen_rot_dir;
                    result.best_rot_deg = rot_deg;
                    result.best_path_scale = path_scale;
                    result.active_range = active_range;
                    result.scaled_range = scaled_range;
                    result.best_score = chosen_score;
                    result.best_path = std::move(follow_path);
                    if (library_options_.plot_path_set) {
                        FillFreeIndices(
                            &result, grid, candidates_, path_scale,
                            scaled_range, lethal_threshold, library_options_);
                        result.feasible_indices = result.free_path_indices;
                    }
                    temporal_state_.valid = true;
                    temporal_state_.rot_dir = chosen_rot_dir;
                    temporal_state_.group_id = chosen_group;
                    temporal_state_.index = chosen_index;
                    return result;
                }
            }
        }

        if (path_scale >= min_path_scale + path_scale_step - 1e-6) {
            path_scale -= path_scale_step;
        } else {
            break;
        }
    }

    result.feasible_indices.clear();
    result.free_path_indices.clear();
    result.scores.assign(candidates_.size(), 0.0);
    return result;
}

std::optional<automsgs::msgs::nav_msgs::Path> IntentPathSelector::Select(
    const map::costmap_2d::Costmap2D& costmap, double joy_dir_deg,
    double joy_speed, const PathSelectContext& context) const {
    CostmapObstacleGrid grid(costmap);
    const auto result = Select(grid, joy_dir_deg, joy_speed, context);
    return result.best_path;
}

std::optional<automsgs::msgs::nav_msgs::Path> IntentPathSelector::Select(
    const automsgs::msgs::map_msgs::OccupancyGrid& grid, double joy_dir_deg,
    double joy_speed, const PathSelectContext& context) const {
    OccupancyGridObstacleGrid obstacle_grid(grid);
    const auto result = Select(obstacle_grid, joy_dir_deg, joy_speed, context);
    return result.best_path;
}

double IntentPathSelector::CompositePathScore(
    double intent_score, const PathObstacleGrid& grid,
    const automsgs::msgs::nav_msgs::Path& world_path,
    const PathLibraryOptions& options, double active_range,
    const PathSelectContext& context, int rot_dir, int group_id,
    const PathTemporalState* last_selection) {
    return ::autonomy::task::teleop::CompositePathScore(
        intent_score, grid, world_path, options, active_range, context,
        rot_dir, group_id, last_selection);
}

void IntentPathSelector::FillFreeIndices(
    PathSelectionResult* result, const PathObstacleGrid& grid,
    const std::vector<PathCandidate>& candidates, double path_scale,
    double scaled_range, int lethal_threshold,
    const PathLibraryOptions& options) {
    ::autonomy::task::teleop::FillFreeIndices(
        result, grid, candidates, path_scale, scaled_range, lethal_threshold,
        options);
}

/**
 * @brief Public wrapper for rot obstacle angle scan
 */
RotObstacleAngles IntentPathSelector::RotObstacleScan(
    const PathObstacleGrid& grid, double path_scale, double vehicle_length,
    double vehicle_width) {
    return ::autonomy::task::teleop::RotObstacleScan(
        grid, path_scale, vehicle_length, vehicle_width);
}

double IntentPathSelector::RotDirToDeg(int rot_dir) {
    return ::autonomy::task::teleop::RotDirToDeg(rot_dir);
}

double IntentPathSelector::RotDirWeight(int rot_dir) {
    return ::autonomy::task::teleop::RotDirWeight(rot_dir);
}

bool IntentPathSelector::JoyRotGate(double joy_dir_deg, int rot_dir,
                                              double dir_threshold,
                                              bool dir_to_vehicle) {
    return ::autonomy::task::teleop::JoyRotGate(
        joy_dir_deg, rot_dir, dir_threshold, dir_to_vehicle);
}

bool IntentPathSelector::RotDirClear(
    int rot_dir, const RotObstacleAngles& angles, bool check_rot_obstacle,
    bool two_way_drive) {
    return ::autonomy::task::teleop::RotDirClear(
        rot_dir, angles, check_rot_obstacle, two_way_drive);
}

/**
 * @brief Public wrapper for safe rotDir search
 */
int IntentPathSelector::BestRotDir(
    double joy_dir_deg, const RotObstacleAngles& angles,
    const PathLibraryOptions& options) {
    return ::autonomy::task::teleop::BestRotDir(joy_dir_deg, angles,
                                                        options);
}

/**
 * @brief Public wrapper for viz fallback scales
 */
void IntentPathSelector::FillPreviewScales(
    PathSelectionResult* result, double joy_speed_norm,
    const PathLibraryOptions& options) {
    ::autonomy::task::teleop::FillPreviewScales(result, joy_speed_norm,
                                                        options);
}

automsgs::msgs::nav_msgs::Path IntentPathSelector::RotateAndScalePath(
    const automsgs::msgs::nav_msgs::Path& path, double rot_deg, double scale) {
    return ::autonomy::task::teleop::RotateAndScalePath(path, rot_deg, scale);
}

double IntentPathSelector::PathIntentScore(double end_dir_deg,
                                                  double joy_dir_deg,
                                                  int rot_dir,
                                                  double dir_weight) {
    return ::autonomy::task::teleop::PathIntentScore(
        end_dir_deg, joy_dir_deg, rot_dir, dir_weight);
}

automsgs::msgs::nav_msgs::Path IntentPathSelector::TrimPathToArcLength(
    const automsgs::msgs::nav_msgs::Path& path, double max_length) {
    return ::autonomy::task::teleop::TrimPathToArcLength(path, max_length);
}

automsgs::msgs::nav_msgs::Path IntentPathSelector::BuildFollowPath(
    const automsgs::msgs::nav_msgs::Path& library_path, double scaled_range,
    double rot_deg, double path_scale) {
    return RotateAndScalePath(TrimPathToArcLength(library_path, scaled_range),
                              rot_deg, path_scale);
}

double IntentPathSelector::GroupHeadingDeg(int group_id,
                                           double max_heading_deg) {
    if (max_heading_deg <= 0.0) {
        return 0.0;
    }
    const double delta_angle = max_heading_deg / 3.0;
    return -max_heading_deg + group_id * delta_angle;
}

bool IntentPathSelector::InRgbdFov(
    const PathCandidate& candidate,
    const automsgs::msgs::nav_msgs::Path& world_path,
    const PathLibraryOptions& options) {
    if (world_path.poses_size() < 2) {
        return false;
    }
    const double half_fov =
        options.rgbd_hfov_deg > 0.0 ? options.rgbd_hfov_deg * 0.5 : 180.0;
    const double ox = options.rgbd_camera_offset_x;
    const double oy = options.rgbd_camera_offset_y;

    if (options.use_polar_spline) {
        const double group_heading =
            GroupHeadingDeg(candidate.group_id, options.max_heading_deg);
        return std::abs(group_heading) <= half_fov + 1e-3;
    }

    return std::abs(EndpointBearing(world_path, ox, oy)) <=
           half_fov + 1e-3;
}

/**
 * @brief Public wrapper for lethal hit counting
 */
int IntentPathSelector::CountLethalHits(
    const PathObstacleGrid& grid,
    const automsgs::msgs::nav_msgs::Path& path,
    double sample_step) {
    int hits = 0;
    const double step = std::max(0.01, sample_step);
    for (int i = 0; i < path.poses_size(); ++i) {
        const double wx = path.poses(i).pose().position().x();
        const double wy = path.poses(i).pose().position().y();

        if (i > 0) {
            const auto& prev = path.poses(i - 1);
            const double px = prev.pose().position().x();
            const double py = prev.pose().position().y();
            const double dx = wx - px;
            const double dy = wy - py;
            const double seg_len = std::hypot(dx, dy);
            if (seg_len > 1e-6) {
                for (double traveled = step; traveled < seg_len;
                     traveled += step) {
                    const double t = traveled / seg_len;
                    if (IsPathBlocked(grid, px + dx * t, py + dy * t)) {
                        ++hits;
                    }
                }
            }
        }

        if (IsPathBlocked(grid, wx, wy)) {
            ++hits;
        }
    }
    return hits;
}

double IntentPathSelector::WrapDeg(double deg) {
    return ::autonomy::task::teleop::WrapDeg(deg);
}

double IntentPathSelector::EndDirectionDeg(double end_x, double end_y) {
    return 2.0 * std::atan2(end_y, end_x) * 180.0 / kPi;
}

}  // namespace autonomy::task::teleop
