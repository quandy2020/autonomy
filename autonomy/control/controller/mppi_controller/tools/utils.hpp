/*
 * Copyright 2025 The Openbot Authors (duyongquan)
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

#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "autonomy/common/math/math.hpp"
#include "autonomy/common/math/math_utils.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/planning_msgs.hpp"
#include "autonomy/control/common/goal_checker_interface.hpp"
#include "autonomy/control/controller/mppi_controller/critic_data.hpp"
#include "autonomy/control/controller/mppi_controller/models/control_sequence.hpp"
#include "autonomy/control/controller/mppi_controller/models/optimizer_settings.hpp"
#include "autonomy/control/controller/mppi_controller/models/path.hpp"
#include "autonomy/map/costmap_2d/cost_values.hpp"
#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/transform/tf2/utils.h"

namespace autonomy {
namespace control {
namespace controller {
namespace mppi {
namespace utils {

static constexpr float kPiF =
    3.141592653589793238462643383279502884e+00F;
static constexpr float kPiF2 = 1.5707963267948966e+00F;

inline float clamp(float lower_bound, float upper_bound, float input) {
    return std::max(lower_bound, std::min(upper_bound, input));
}

struct Pose2D {
    float x{0.0f};
    float y{0.0f};
    float theta{0.0f};
};

inline models::Path toTensor(const commsgs::planning_msgs::Path& path) {
    models::Path result;
    result.reset(path.poses.size());
    for (size_t i = 0; i < path.poses.size(); ++i) {
        result.x(static_cast<Eigen::Index>(i)) = path.poses[i].pose.position.x;
        result.y(static_cast<Eigen::Index>(i)) = path.poses[i].pose.position.y;
        result.yaws(static_cast<Eigen::Index>(i)) =
            static_cast<float>(transform::tf2::getYaw(path.poses[i].pose.orientation));
    }
    return result;
}

inline commsgs::geometry_msgs::Pose getLastPathPose(const models::Path& path) {
    const Eigen::Index last = path.x.size() - 1;
    commsgs::geometry_msgs::Pose pose;
    pose.position.x = path.x(last);
    pose.position.y = path.y(last);
    pose.position.z = 0.0f;
    const float yaw = path.yaws(last);
    pose.orientation.z = std::sin(yaw / 2.0f);
    pose.orientation.w = std::cos(yaw / 2.0f);
    return pose;
}

inline commsgs::geometry_msgs::Pose getCriticGoal(const CriticData& data,
                                                  bool enforce_path_inversion) {
    if (enforce_path_inversion) {
        return getLastPathPose(data.path);
    }
    return data.goal;
}

inline bool withinPositionGoalTolerance(
    common::GoalChecker* goal_checker,
    const commsgs::geometry_msgs::Pose& robot,
    const commsgs::geometry_msgs::Pose& goal) {
    if (!goal_checker) {
        return false;
    }
    commsgs::geometry_msgs::Pose pose_tolerance;
    commsgs::geometry_msgs::Twist vel_tolerance;
    if (!goal_checker->GetTolerances(pose_tolerance, vel_tolerance)) {
        return false;
    }
    const float tol = pose_tolerance.position.x;
    const float dx = robot.position.x - goal.position.x;
    const float dy = robot.position.y - goal.position.y;
    return (dx * dx + dy * dy) < (tol * tol);
}

inline bool withinPositionGoalTolerance(float pose_tolerance,
                                        const commsgs::geometry_msgs::Pose& robot,
                                        const commsgs::geometry_msgs::Pose& goal) {
    const float dx = robot.position.x - goal.position.x;
    const float dy = robot.position.y - goal.position.y;
    return (dx * dx + dy * dy) < (pose_tolerance * pose_tolerance);
}

template <typename T>
auto normalize_angles(const T& angles) {
    return (angles + kPiF)
        .unaryExpr([](float x) {
            float remainder = std::fmod(x, 2.0f * kPiF);
            return remainder < 0.0f ? remainder + kPiF : remainder - kPiF;
        });
}

template <typename F, typename T>
auto shortest_angular_distance(const F& from, const T& to) {
    return normalize_angles(to - from);
}

inline size_t findPathFurthestReachedPoint(const CriticData& data) {
    const int traj_cols = data.trajectories.x.cols();
    const auto traj_x = data.trajectories.x.col(traj_cols - 1);
    const auto traj_y = data.trajectories.y.col(traj_cols - 1);

    const auto dx =
        (data.path.x.transpose()).replicate(traj_x.rows(), 1).colwise() - traj_x;
    const auto dy =
        (data.path.y.transpose()).replicate(traj_y.rows(), 1).colwise() - traj_y;
    const auto dists = dx * dx + dy * dy;

    int max_id_by_trajectories = 0;
    const size_t n_rows = dists.rows();
    const size_t n_cols = dists.cols();
    for (size_t i = 0; i < n_rows; ++i) {
        int min_id_by_path = 0;
        float min_distance_by_path = std::numeric_limits<float>::max();
        for (size_t j = static_cast<size_t>(max_id_by_trajectories); j < n_cols;
             ++j) {
            const float cur_dist = dists(static_cast<Eigen::Index>(i),
                                       static_cast<Eigen::Index>(j));
            if (cur_dist < min_distance_by_path) {
                min_distance_by_path = cur_dist;
                min_id_by_path = static_cast<int>(j);
            }
        }
        max_id_by_trajectories = std::max(max_id_by_trajectories, min_id_by_path);
    }
    return static_cast<size_t>(max_id_by_trajectories);
}

inline void setPathFurthestPointIfNotSet(CriticData& data) {
    if (!data.furthest_reached_path_point) {
        data.furthest_reached_path_point = findPathFurthestReachedPoint(data);
    }
}

inline void findPathCosts(
    CriticData& data,
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper) {
    auto* costmap = costmap_wrapper->getCostmap();
    unsigned int map_x = 0;
    unsigned int map_y = 0;
    const size_t path_segments_count = data.path.x.size();
    data.path_pts_valid = std::vector<bool>(path_segments_count, false);
    const bool tracking_unknown =
        costmap_wrapper->getLayeredCostmap()->isTrackingUnknown();
    for (size_t idx = 0; idx < path_segments_count; ++idx) {
        if (!costmap->worldToMap(data.path.x(static_cast<Eigen::Index>(idx)),
                                 data.path.y(static_cast<Eigen::Index>(idx)),
                                 map_x, map_y)) {
            (*data.path_pts_valid)[idx] = false;
            continue;
        }
        switch (costmap->getCost(map_x, map_y)) {
            case map::costmap_2d::LETHAL_OBSTACLE:
            case map::costmap_2d::INSCRIBED_INFLATED_OBSTACLE:
                (*data.path_pts_valid)[idx] = false;
                continue;
            case map::costmap_2d::NO_INFORMATION:
                (*data.path_pts_valid)[idx] = tracking_unknown;
                continue;
            default:
                (*data.path_pts_valid)[idx] = true;
                break;
        }
    }
}

inline void setPathCostsIfNotSet(
    CriticData& data,
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper) {
    if (!data.path_pts_valid) {
        findPathCosts(data, costmap_wrapper);
    }
}

inline float posePointAngle(const commsgs::geometry_msgs::Pose& pose,
                            double point_x, double point_y,
                            bool forward_preference) {
    const float pose_x = pose.position.x;
    const float pose_y = pose.position.y;
    const float pose_yaw =
        static_cast<float>(transform::tf2::getYaw(pose.orientation));
    const float yaw =
        atan2f(static_cast<float>(point_y) - pose_y,
               static_cast<float>(point_x) - pose_x);
    if (!forward_preference) {
        const float a = std::fabs(static_cast<float>(
            ::autonomy::common::NormalizeAngleDifference(yaw - pose_yaw)));
        const float reversed_yaw = static_cast<float>(
            ::autonomy::common::math::NormalizeAngle(
                static_cast<double>(pose_yaw + kPiF)));
        const float b = std::fabs(static_cast<float>(
            ::autonomy::common::NormalizeAngleDifference(yaw - reversed_yaw)));
        return std::min(a, b);
    }
    return std::fabs(
        static_cast<float>(::autonomy::common::NormalizeAngleDifference(
            yaw - pose_yaw)));
}

inline float posePointAngle(const commsgs::geometry_msgs::Pose& pose,
                            double point_x, double point_y, double point_yaw) {
    const float pose_x = pose.position.x;
    const float pose_y = pose.position.y;
    const float pose_yaw =
        static_cast<float>(transform::tf2::getYaw(pose.orientation));
    float yaw = atan2f(static_cast<float>(point_y) - pose_y,
                       static_cast<float>(point_x) - pose_x);
    if (std::fabs(static_cast<float>(
            ::autonomy::common::NormalizeAngleDifference(
                yaw - static_cast<float>(point_yaw)))) > kPiF2) {
        yaw = static_cast<float>(
            ::autonomy::common::math::NormalizeAngle(static_cast<double>(yaw + kPiF)));
    }
    return std::fabs(static_cast<float>(
        ::autonomy::common::NormalizeAngleDifference(yaw - pose_yaw)));
}

inline unsigned int findClosestPathPt(const std::vector<float>& path_integrated,
                                      float target, unsigned int start) {
    const auto it = std::lower_bound(path_integrated.begin() + start,
                                     path_integrated.end(), target);
    if (it == path_integrated.end()) {
        return path_integrated.size() - 1;
    }
    if (it == path_integrated.begin()) {
        return 0;
    }
    const auto prev = it - 1;
    return static_cast<unsigned int>(
        (target - *prev) < (*it - target) ? std::distance(path_integrated.begin(),
                                                          prev)
                                          : std::distance(path_integrated.begin(),
                                                          it));
}

inline void shiftColumnsByOnePlace(Eigen::Ref<Eigen::ArrayXXf> array,
                                   int direction) {
    if (direction > 0) {
        for (int i = array.cols() - 1; i > 0; --i) {
            array.col(i) = array.col(i - 1);
        }
    } else if (direction < 0) {
        for (int i = 0; i < array.cols() - 1; ++i) {
            array.col(i) = array.col(i + 1);
        }
    }
}

inline void shiftVectorByOnePlace(Eigen::Ref<Eigen::ArrayXf> array,
                                  int direction) {
    if (direction > 0) {
        for (int i = array.size() - 1; i > 0; --i) {
            array(i) = array(i - 1);
        }
    } else if (direction < 0) {
        for (int i = 0; i < array.size() - 1; ++i) {
            array(i) = array(i + 1);
        }
    }
}

inline Eigen::ArrayXf normalize_yaws_between_points(
    const Eigen::Ref<const Eigen::ArrayXf>& last_yaws,
    const Eigen::ArrayXf& yaw_between_points) {
    Eigen::ArrayXf out = yaw_between_points;
    for (int i = 0; i < out.size(); ++i) {
        const float last = last_yaws(i);
        float y = out(i);
        if (std::fabs(static_cast<float>(
                ::autonomy::common::NormalizeAngleDifference(y - last))) > kPiF2) {
            y = static_cast<float>(
                ::autonomy::common::math::NormalizeAngle(static_cast<double>(y + kPiF)));
        }
        out(i) = y;
    }
    return out;
}

inline void savitskyGolayFilter(
    models::ControlSequence& control_sequence,
    std::array<models::Control, 4>& control_history,
    const models::OptimizerSettings& settings) {
    Eigen::Array<float, 9, 1> filter = {-21.0f, 14.0f, 39.0f, 54.0f, 59.0f,
                                        54.0f, 39.0f, 14.0f, -21.0f};
    filter /= 231.0f;

    const unsigned int num_sequences =
        static_cast<unsigned int>(control_sequence.vx.size()) - 1;
    if (num_sequences < 20) {
        return;
    }

    auto applyFilter = [&](const Eigen::Array<float, 9, 1>& data) -> float {
        return (data * filter).eval().sum();
    };

    auto applyFilterOverAxis = [&](Eigen::ArrayXf& sequence,
                                   const Eigen::ArrayXf& initial_sequence,
                                   float hist_0, float hist_1, float hist_2,
                                   float hist_3) {
        float pt_m4 = hist_0;
        float pt_m3 = hist_1;
        float pt_m2 = hist_2;
        float pt_m1 = hist_3;
        float pt = initial_sequence(0);
        float pt_p1 = initial_sequence(1);
        float pt_p2 = initial_sequence(2);
        float pt_p3 = initial_sequence(3);
        float pt_p4 = initial_sequence(4);

        for (unsigned int idx = 0; idx != num_sequences; ++idx) {
            sequence(static_cast<Eigen::Index>(idx)) =
                applyFilter({pt_m4, pt_m3, pt_m2, pt_m1, pt, pt_p1, pt_p2,
                             pt_p3, pt_p4});
            pt_m4 = pt_m3;
            pt_m3 = pt_m2;
            pt_m2 = pt_m1;
            pt_m1 = pt;
            pt = pt_p1;
            pt_p1 = pt_p2;
            pt_p2 = pt_p3;
            pt_p3 = pt_p4;
            if (idx + 5 < num_sequences) {
                pt_p4 = initial_sequence(static_cast<Eigen::Index>(idx + 5));
            } else {
                pt_p4 = initial_sequence(static_cast<Eigen::Index>(num_sequences));
            }
        }
    };

    const models::ControlSequence initial = control_sequence;
    applyFilterOverAxis(control_sequence.vx, initial.vx, control_history[0].vx,
                        control_history[1].vx, control_history[2].vx,
                        control_history[3].vx);
    applyFilterOverAxis(control_sequence.vy, initial.vy, control_history[0].vy,
                        control_history[1].vy, control_history[2].vy,
                        control_history[3].vy);
    applyFilterOverAxis(control_sequence.wz, initial.wz, control_history[0].wz,
                        control_history[1].wz, control_history[2].wz,
                        control_history[3].wz);

    const unsigned int offset = settings.shift_control_sequence ? 1u : 0u;
    control_history[0] = control_history[1];
    control_history[1] = control_history[2];
    control_history[2] = control_history[3];
    control_history[3] = {control_sequence.vx(static_cast<Eigen::Index>(offset)),
                          control_sequence.vy(static_cast<Eigen::Index>(offset)),
                          control_sequence.wz(static_cast<Eigen::Index>(offset))};
}

}  // namespace utils
}  // namespace mppi
}  // namespace controller
}  // namespace control
}  // namespace autonomy
