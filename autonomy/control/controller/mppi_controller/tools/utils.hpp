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
 #include <chrono>
 #include <limits>
 #include <memory>
 #include <string>
 #include <vector>
 
 #include "autonomy/common/macros.hpp"
 #include "autonomy/common/math/angle.hpp"
 #include "autonomy/common/math/math.hpp"
 
 #include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/vector3.pb.h>
#include <automsgs/msgs/std_msgs/color_rgba.pb.h>
#include <automsgs/msgs/std_msgs/header.pb.h>
#include <automsgs/msgs/visualization_msgs/marker.pb.h>
#include <automsgs/msgs/planning_msgs/planning_msgs.pb.h>
#include <automsgs/msgs/time_utils.hpp>
 
 
 #include "autonomy/control/controller/mppi_controller/critic_data.hpp"
 #include "autonomy/control/controller/mppi_controller/models/control_sequence.hpp"
 #include "autonomy/control/controller/mppi_controller/models/optimizer_settings.hpp"
 #include "autonomy/control/controller/mppi_controller/models/path.hpp"
 #include "autonomy/map/costmap_2d/cost_values.hpp"
 #include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
 #include "autonomy/transform/tf2/utils.h"
 
 #define M_PIF 3.141592653589793238462643383279502884e+00F
 #define M_PIF_2 1.5707963267948966e+00F
 
 namespace autonomy {
 namespace control {
 namespace controller {
 namespace mppi_controller {
 namespace tools {
 
 /**
  * @brief Convert data into pose
  * @param x X position
  * @param y Y position
  * @param z Z position
  * @return Pose object
  */
 inline automsgs::msgs::geometry_msgs::Pose createPose(double x, double y, double z) {
     automsgs::msgs::geometry_msgs::Pose pose;
     pose.mutable_position()->set_x(x);
     pose.mutable_position()->set_y(y);
     pose.mutable_position()->set_z(z);
     pose.mutable_orientation()->set_w(1);
     pose.mutable_orientation()->set_x(0);
     pose.mutable_orientation()->set_y(0);
     pose.mutable_orientation()->set_z(0);
     return pose;
 }
 
 /**
  * @brief Convert data into scale
  * @param x X scale
  * @param y Y scale
  * @param z Z scale
  * @return Scale object
  */
 inline automsgs::msgs::geometry_msgs::Vector3 createScale(double x, double y,
                                                    double z) {
     automsgs::msgs::geometry_msgs::Vector3 scale;
     scale.set_x(x);
     scale.set_y(y);
     scale.set_z(z);
     return scale;
 }
 
 /**
  * @brief Convert data into color
  * @param r Red component
  * @param g Green component
  * @param b Blue component
  * @param a Alpha component (transparency)
  * @return Color object
  */
 inline automsgs::msgs::std_msgs::ColorRGBA createColor(float r, float g, float b,
                                                 float a) {
     automsgs::msgs::std_msgs::ColorRGBA color;
     color.set_r(r);
     color.set_g(g);
     color.set_b(b);
     color.set_a(a);
     return color;
 }
 
 /**
  * @brief Convert data into a Maarker
  * @param id Marker ID
  * @param pose Marker pose
  * @param scale Marker scale
  * @param color Marker color
  * @param frame Reference frame to use
  * @return Visualization Marker
  */
 inline automsgs::msgs::visualization_msgs::Marker createMarker(
     int id, const automsgs::msgs::geometry_msgs::Pose& pose,
     const automsgs::msgs::geometry_msgs::Vector3& scale,
     const automsgs::msgs::std_msgs::ColorRGBA& color, const std::string& frame_id,
     const std::string& ns) {
     using automsgs::msgs::visualization_msgs::Marker;
     automsgs::msgs::visualization_msgs::Marker marker;
     marker.mutable_header()->set_frame_id(frame_id);
     marker.mutable_header()->mutable_stamp()->Clear();
     marker.set_ns(ns);
     marker.set_id(id);
     marker.set_type(automsgs::msgs::visualization_msgs::Marker::SPHERE);    // SPHERE
     marker.set_action(automsgs::msgs::visualization_msgs::Marker::ADD);  // ADD

     *marker.mutable_pose() = pose;
     *marker.mutable_scale() = scale;
     *marker.mutable_color() = color;
     return marker;
 }
 
 /**
  * @brief Convert data into TwistStamped
  * @param vx X velocity
  * @param wz Angular velocity
  * @param stamp Timestamp
  * @param frame Reference frame to use
  */
 inline automsgs::msgs::geometry_msgs::TwistStamped toTwistStamped(
     float vx, float wz, const automsgs::msgs::builtin_interfaces::Time& stamp,
     const std::string& frame) {
     automsgs::msgs::geometry_msgs::TwistStamped twist;
     twist.mutable_header()->set_frame_id( frame);
     *twist.mutable_header()->mutable_stamp() = stamp;
     twist.mutable_twist()->mutable_linear()->set_x(vx);
     twist.mutable_twist()->mutable_angular()->set_z(wz);
 
     return twist;
 }
 
 /**
  * @brief Convert data into TwistStamped
  * @param vx X velocity
  * @param vy Y velocity
  * @param wz Angular velocity
  * @param stamp Timestamp
  * @param frame Reference frame to use
  */
 inline automsgs::msgs::geometry_msgs::TwistStamped toTwistStamped(
     float vx, float vy, float wz,
     const automsgs::msgs::builtin_interfaces::Time& stamp, const std::string& frame) {
     auto twist = toTwistStamped(vx, wz, stamp, frame);
     twist.mutable_twist()->mutable_linear()->set_y(vy);
 
     return twist;
 }
 
 // TODO: Trajectory type not available in planning_msgs, need to define or use
 // alternative inline std::unique_ptr<automsgs::msgs::planning_msgs::Trajectory>
 // toTrajectoryMsg(
 //     const Eigen::ArrayXXf& trajectory,
 //     const models::ControlSequence& control_sequence, const double& model_dt,
 //     const automsgs::msgs::std_msgs::Header& header) {
 //     auto trajectory_msg =
 //     std::make_unique<automsgs::msgs::planning_msgs::Trajectory>();
 //     trajectory_msg->header = header;
 //     trajectory_msg->points.resize(trajectory.rows());
 //
 //     for (int i = 0; i < trajectory.rows(); ++i) {
 //         auto& curr_pt = trajectory_msg->points[i];
 //         curr_pt.time_from_start =
 //         automsgs::msgs::builtin_interfaces::DurationFromSeconds(i * model_dt);
 //         curr_pt.mutable_pose()->mutable_position()->set_x(trajectory(i, 0));
 //         curr_pt.mutable_pose()->mutable_position()->set_y(trajectory(i, 1));
 //         tf2::Quaternion quat;
 //         quat.setRPY(0.0, 0.0, trajectory(i, 2));
 //         curr_pt.pose().orientation = tf2::toMsg(quat);
 //         curr_pt.velocity.mutable_linear()->set_x(control_sequence.vx(i));
 //         curr_pt.velocity.mutable_angular()->set_z(control_sequence.wz(i));
 //         if (control_sequence.vy.size() > 0) {
 //             curr_pt.velocity.mutable_linear()->set_y(control_sequence.vy(i));
 //         }
 //     }
 //
 //     return trajectory_msg;
 // }
 
 /**
  * @brief Convert path to a tensor
  * @param path Path to convert
  * @return Path tensor
  */
 inline models::Path toTensor(const automsgs::msgs::planning_msgs::Path& path) {
     auto result = models::Path{};
     result.reset(path.poses_size());
 
     for (size_t i = 0; i < path.poses_size(); ++i) {
         result.x(i) = path.poses(i).pose().position().x();
         result.y(i) = path.poses(i).pose().position().y();
         result.yaws(i) =
             autonomy::transform::tf2::getYaw(path.poses(i).pose().orientation());
     }
 
     return result;
 }
 
 /**
  * @brief Get the last pose from a path
  * @param path Reference to the path
  * @return automsgs::msgs::geometry_msgs::Pose Last pose in the path
  */
 inline automsgs::msgs::geometry_msgs::Pose getLastPathPose(const models::Path& path) {
     const unsigned int path_last_idx = path.x.size() - 1;
 
     auto last_orientation = path.yaws(path_last_idx);
 
     autonomy::transform::tf2::Quaternion pose_orientation;
     pose_orientation.setRPY(0.0, 0.0, last_orientation);
 
     automsgs::msgs::geometry_msgs::Pose pathPose;
     pathPose.mutable_position()->set_x(path.x(path_last_idx));
     pathPose.mutable_position()->set_y(path.y(path_last_idx));
     pathPose.mutable_orientation()->set_x(pose_orientation.x());
     pathPose.mutable_orientation()->set_y(pose_orientation.y());
     pathPose.mutable_orientation()->set_z(pose_orientation.z());
     pathPose.mutable_orientation()->set_w(pose_orientation.w());
 
     return pathPose;
 }
 
 /**
  * @brief Get the target pose to be evaluated by the critic
  * @param data Data to use
  * @param enforce_path_inversion True to return the cusp point (last pose of the
  * path) instead of the original goal
  * @return automsgs::msgs::geometry_msgs::Pose Target pose for the critic
  */
 inline automsgs::msgs::geometry_msgs::Pose getCriticGoal(const CriticData& data,
                                                   bool enforce_path_inversion) {
     if (enforce_path_inversion) {
         return getLastPathPose(data.path);
     } else {
         return data.goal;
     }
 }
 
 /**
  * @brief Check if the robot pose is within the Goal Checker's tolerances to
  * goal
  * @param global_checker Pointer to the goal checker
  * @param robot Pose of robot
  * @param goal Goal pose
  * @return bool If robot is within goal checker tolerances to the goal
  */
 inline bool withinPositionGoalToleranceWithChecker(
     common::GoalChecker* goal_checker,
     const automsgs::msgs::geometry_msgs::Pose& robot,
     const automsgs::msgs::geometry_msgs::Pose& goal) {
     if (goal_checker) {
         automsgs::msgs::geometry_msgs::Pose pose_tolerance;
         automsgs::msgs::geometry_msgs::Twist velocity_tolerance;
         goal_checker->GetTolerances(pose_tolerance, velocity_tolerance);
 
         const auto pose_tolerance_sq =
             pose_tolerance.position().x() * pose_tolerance.position().x();
 
         auto dx = robot.position().x() - goal.position().x();
         auto dy = robot.position().y() - goal.position().y();
 
         auto dist_sq = dx * dx + dy * dy;
 
         if (dist_sq < pose_tolerance_sq) {
             return true;
         }
     }
 
     return false;
 }
 
 /**
  * @brief Check if the robot pose is within tolerance to the goal
  * @param pose_tolerance Pose tolerance to use
  * @param robot Pose of robot
  * @param goal Goal pose
  * @return bool If robot is within tolerance to the goal
  */
 inline bool withinPositionGoalTolerance(
     float pose_tolerance, const automsgs::msgs::geometry_msgs::Pose& robot,
     const automsgs::msgs::geometry_msgs::Pose& goal) {
     const double& dist_sq = std::pow(goal.position().x() - robot.position().x(), 2) +
                             std::pow(goal.position().y() - robot.position().y(), 2);
 
     const float pose_tolerance_sq = pose_tolerance * pose_tolerance;
 
     if (dist_sq < pose_tolerance_sq) {
         return true;
     }
 
     return false;
 }
 
 /**
  * @brief normalize_angle (single value version)
  * Normalizes a single angle to be -M_PIF circle to +M_PIF circle
  * It takes and returns radians.
  * @param angle Single angle to normalize
  * @return normalized angle
  */
 inline float normalize_angle(float angle) {
     float remainder = std::fmod(angle + M_PIF, 2.0f * M_PIF);
     return remainder < 0.0f ? remainder + M_PIF : remainder - M_PIF;
 }
 
 /**
  * @brief normalize
  * Normalizes the angle to be -M_PIF circle to +M_PIF circle
  * It takes and returns radians.
  * @param angles Angles to normalize
  * @return normalized angles
  */
 template <typename T>
 auto normalize_angles(const T& angles) {
     if constexpr (std::is_arithmetic_v<T>) {
         // Scalar version
         float remainder =
             std::fmod(static_cast<float>(angles) + M_PIF, 2.0f * M_PIF);
         return remainder < 0.0f ? remainder + M_PIF : remainder - M_PIF;
     } else {
         // Eigen array/matrix version
         return (angles + M_PIF).unaryExpr([&](const float x) {
             float remainder = std::fmod(x, 2.0f * M_PIF);
             return remainder < 0.0f ? remainder + M_PIF : remainder - M_PIF;
         });
     }
 }
 
 /**
  * @brief shortest_angular_distance
  *
  * Given 2 angles, this returns the shortest angular
  * difference.  The inputs and outputs are of course radians.
  *
  * The result
  * would always be -pi <= result <= pi.  Adding the result
  * to "from" will always get you an equivalent angle to "to".
  * @param from Start angle
  * @param to End angle
  * @return Shortest distance between angles
  */
 template <typename F, typename T>
 auto shortest_angular_distance(const F& from, const T& to) {
     return normalize_angles(to - from);
 }
 
 /**
  * @brief Evaluate furthest point idx of data.path which is
  * nearest to some trajectory in data.trajectories
  * @param data Data to use
  * @return Idx of furthest path point reached by a set of trajectories
  */
 inline size_t findPathFurthestReachedPoint(const CriticData& data) {
     int traj_cols = data.trajectories.x.cols();
     const auto traj_x = data.trajectories.x.col(traj_cols - 1);
     const auto traj_y = data.trajectories.y.col(traj_cols - 1);
 
     const auto dx =
         (data.path.x.transpose()).replicate(traj_x.rows(), 1).colwise() -
         traj_x;
     const auto dy =
         (data.path.y.transpose()).replicate(traj_y.rows(), 1).colwise() -
         traj_y;
 
     const auto dists = dx * dx + dy * dy;
 
     int max_id_by_trajectories = 0, min_id_by_path = 0;
     float min_distance_by_path = std::numeric_limits<float>::max();
     size_t n_rows = dists.rows();
     size_t n_cols = dists.cols();
     for (size_t i = 0; i != n_rows; i++) {
         min_id_by_path = 0;
         min_distance_by_path = std::numeric_limits<float>::max();
         for (size_t j = max_id_by_trajectories; j != n_cols; j++) {
             const float cur_dist = dists(i, j);
             if (cur_dist < min_distance_by_path) {
                 min_distance_by_path = cur_dist;
                 min_id_by_path = j;
             }
         }
         max_id_by_trajectories =
             std::max(max_id_by_trajectories, min_id_by_path);
     }
     return max_id_by_trajectories;
 }
 
 /**
  * @brief evaluate path furthest point if it is not set
  * @param data Data to use
  */
 inline void setPathFurthestPointIfNotSet(CriticData& data) {
     if (!data.furthest_reached_path_point) {
         data.furthest_reached_path_point = findPathFurthestReachedPoint(data);
     }
 }
 
 /**
  * @brief evaluate path costs
  * @param data Data to use
  */
 inline void findPathCosts(
     CriticData& data,
     std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper) {
     auto* costmap = costmap_wrapper->getCostmap();
     unsigned int map_x, map_y;
     const size_t path_segments_count = data.path.x.size() - 1;
     data.path_pts_valid = std::vector<bool>(path_segments_count, false);
     const bool tracking_unknown =
         costmap_wrapper->getLayeredCostmap()->isTrackingUnknown();
     for (unsigned int idx = 0; idx < path_segments_count; idx++) {
         if (!costmap->worldToMap(data.path.x(idx), data.path.y(idx), map_x,
                                  map_y)) {
             (*data.path_pts_valid)[idx] = false;
             continue;
         }
 
         switch (costmap->getCost(map_x, map_y)) {
             case (map::costmap_2d::LETHAL_OBSTACLE):
                 (*data.path_pts_valid)[idx] = false;
                 continue;
             case (map::costmap_2d::INSCRIBED_INFLATED_OBSTACLE):
                 (*data.path_pts_valid)[idx] = false;
                 continue;
             case (map::costmap_2d::NO_INFORMATION):
                 (*data.path_pts_valid)[idx] = tracking_unknown ? true : false;
                 continue;
         }
 
         (*data.path_pts_valid)[idx] = true;
     }
 }
 
 /**
  * @brief evaluate path costs if it is not set
  * @param data Data to use
  */
 inline void setPathCostsIfNotSet(
     CriticData& data,
     std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper) {
     if (!data.path_pts_valid) {
         findPathCosts(data, costmap_wrapper);
     }
 }
 
 /**
  * @brief evaluate angle from pose (have angle) to point (no angle)
  * @param pose pose
  * @param point_x Point to find angle relative to X axis
  * @param point_y Point to find angle relative to Y axis
  * @param forward_preference If reversing direction is valid
  * @return Angle between two points
  */
 inline float posePointAngle(const automsgs::msgs::geometry_msgs::Pose& pose,
                             double point_x, double point_y,
                             bool forward_preference) {
     float pose_x = pose.position().x();
     float pose_y = pose.position().y();
     float pose_yaw = autonomy::transform::tf2::getYaw(pose.orientation());
 
     float yaw = atan2f(point_y - pose_y, point_x - pose_x);
 
     // If no preference for forward, return smallest angle either in heading or
     // 180 of heading
     if (!forward_preference) {
         return std::min(fabs(shortest_angular_distance(yaw, pose_yaw)),
                         fabs(shortest_angular_distance(
                             yaw, normalize_angle(pose_yaw + M_PIF))));
     }
 
     return fabs(shortest_angular_distance(yaw, pose_yaw));
 }
 
 /**
  * @brief evaluate angle from pose (have angle) to point (no angle)
  * @param pose pose
  * @param point_x Point to find angle relative to X axis
  * @param point_y Point to find angle relative to Y axis
  * @param point_yaw Yaw of the point to consider along Z axis
  * @return Angle between two points
  */
 inline float posePointAngle(const automsgs::msgs::geometry_msgs::Pose& pose,
                             double point_x, double point_y, double point_yaw) {
     float pose_x = static_cast<float>(pose.position().x());
     float pose_y = static_cast<float>(pose.position().y());
     float pose_yaw =
         static_cast<float>(autonomy::transform::tf2::getYaw(pose.orientation()));
 
     float yaw = atan2f(static_cast<float>(point_y) - pose_y,
                        static_cast<float>(point_x) - pose_x);
 
     if (fabs(shortest_angular_distance(yaw, static_cast<float>(point_yaw))) >
         M_PIF_2) {
         yaw = normalize_angle(yaw + M_PIF);
     }
 
     return fabs(shortest_angular_distance(yaw, pose_yaw));
 }
 
 /**
  * @brief Apply Savisky-Golay filter to optimal trajectory
  * @param control_sequence Sequence to apply filter to
  * @param control_history Recent set of controls for edge-case handling
  * @param Settings Settings to use
  */
 inline void savitskyGolayFilter(models::ControlSequence& control_sequence,
                                 std::array<models::Control, 4>& control_history,
                                 const models::OptimizerSettings& settings) {
     Eigen::Array<float, 9, 1> filter;
     if (settings.sgf_order == 1) {
         filter = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
                   1.0f, 1.0f, 1.0f, 1.0f};
         filter /= 9.0f;
     } else {
         filter = {-21.0f, 14.0f, 39.0f, 54.0f, 59.0f,
                   54.0f,  39.0f, 14.0f, -21.0f};
         filter /= 231.0f;
     }
 
     // Too short to smooth meaningfully
     const unsigned int num_sequences = control_sequence.vx.size() - 1;
     if (num_sequences < 20) {
         return;
     }
 
     auto applyFilter = [&](const Eigen::Array<float, 9, 1>& data) -> float {
         return (data * filter).eval().sum();
     };
 
     auto applyFilterOverAxis =
         [&](Eigen::ArrayXf& sequence, const Eigen::ArrayXf& initial_sequence,
             const float hist_0, const float hist_1, const float hist_2,
             const float hist_3) -> void {
         float pt_m4 = hist_0;
         float pt_m3 = hist_1;
         float pt_m2 = hist_2;
         float pt_m1 = hist_3;
         float pt = initial_sequence(0);
         float pt_p1 = initial_sequence(1);
         float pt_p2 = initial_sequence(2);
         float pt_p3 = initial_sequence(3);
         float pt_p4 = initial_sequence(4);
 
         for (unsigned int idx = 0; idx != num_sequences; idx++) {
             sequence(idx) = applyFilter(
                 {pt_m4, pt_m3, pt_m2, pt_m1, pt, pt_p1, pt_p2, pt_p3, pt_p4});
             pt_m4 = pt_m3;
             pt_m3 = pt_m2;
             pt_m2 = pt_m1;
             pt_m1 = pt;
             pt = pt_p1;
             pt_p1 = pt_p2;
             pt_p2 = pt_p3;
             pt_p3 = pt_p4;
 
             if (idx + 5 < num_sequences) {
                 pt_p4 = initial_sequence(idx + 5);
             } else {
                 // Return the last point
                 pt_p4 = initial_sequence(num_sequences);
             }
         }
     };
 
     // Filter trajectories
     const models::ControlSequence initial_control_sequence = control_sequence;
     applyFilterOverAxis(control_sequence.vx, initial_control_sequence.vx,
                         control_history[0].vx, control_history[1].vx,
                         control_history[2].vx, control_history[3].vx);
     applyFilterOverAxis(control_sequence.vy, initial_control_sequence.vy,
                         control_history[0].vy, control_history[1].vy,
                         control_history[2].vy, control_history[3].vy);
     applyFilterOverAxis(control_sequence.wz, initial_control_sequence.wz,
                         control_history[0].wz, control_history[1].wz,
                         control_history[2].wz, control_history[3].wz);
 
     // Update control history
     unsigned int offset = settings.shift_control_sequence ? 1 : 0;
     control_history[0] = control_history[1];
     control_history[1] = control_history[2];
     control_history[2] = control_history[3];
     control_history[3] = {control_sequence.vx(offset),
                           control_sequence.vy(offset),
                           control_sequence.wz(offset)};
 }
 
 /**
  * @brief Find the iterator of the first pose at which there is an inversion on
  * the path,
  * @param path to check for inversion
  * @return the first point after the inversion found in the path
  */
 inline unsigned int findFirstPathInversion(automsgs::msgs::planning_msgs::Path& path) {
     // At least 3 poses for a possible inversion
     if (path.poses_size() < 3) {
         return path.poses_size();
     }
 
     // Iterating through the path to determine the position of the path
     // inversion
     for (unsigned int idx = 1; idx < path.poses_size() - 1; ++idx) {
         // We have two vectors for the dot product OA and AB. Determining the
         // vectors.
         float oa_x = path.poses(idx).pose().position().x() -
                      path.poses(idx - 1).pose().position().x();
         float oa_y = path.poses(idx).pose().position().y() -
                      path.poses(idx - 1).pose().position().y();
         float ab_x = path.poses(idx + 1).pose().position().x() -
                      path.poses(idx).pose().position().x();
         float ab_y = path.poses(idx + 1).pose().position().y() -
                      path.poses(idx).pose().position().y();
 
         // Checking for the existence of cusp, in the path, using the dot
         // product.
         float dot_product = (oa_x * ab_x) + (oa_y * ab_y);
         if (dot_product < 0.0f) {
             return idx + 1;
         }
     }
 
     return path.poses_size();
 }
 
 /**
  * @brief Find and remove poses after the first inversion in the path
  * @param path to check for inversion
  * @return The location of the inversion, return 0 if none exist
  */
 inline unsigned int removePosesAfterFirstInversion(
     automsgs::msgs::planning_msgs::Path& path) {
     automsgs::msgs::planning_msgs::Path cropped_path = path;
     const unsigned int first_after_inversion =
         findFirstPathInversion(cropped_path);
     if (first_after_inversion == path.poses_size()) {
         return 0u;
     }
 
     cropped_path.mutable_poses()->DeleteSubrange(
         first_after_inversion,
         cropped_path.poses_size() - first_after_inversion);
     path = cropped_path;
     return first_after_inversion;
 }
 
 /**
  * @brief Compare to trajectory points to find closest path point along
  * integrated distances
  * @param vec Vect to check
  * @return dist Distance to look for
  * @return init Starting index to indec from
  */
 inline unsigned int findClosestPathPt(const std::vector<float>& vec,
                                       const float dist,
                                       const unsigned int init = 0u) {
     float distim1 = init != 0u
                         ? vec[init]
                         : 0.0f;  // First is 0, no accumulated distance yet
     float disti = 0.0f;
     const unsigned int size = vec.size();
     for (unsigned int i = init + 1; i != size; i++) {
         disti = vec[i];
         if (disti > dist) {
             if (i > 0 && dist - distim1 < disti - dist) {
                 return i - 1;
             }
             return i;
         }
         distim1 = disti;
     }
     return size - 1;
 }
 
 // A struct to hold pose data in floating point resolution
 struct Pose2D {
     float x, y, theta;
 };
 
 /**
  * @brief Shift the columns of a 2D Eigen Array or scalar values of
  *    1D Eigen Array by 1 place.
  * @param e Eigen Array
  * @param direction direction in which Array will be shifted.
  *     1 for shift in right direction and -1 for left direction.
  */
 inline void shiftColumnsByOnePlace(Eigen::Ref<Eigen::ArrayXXf> e,
                                    int direction) {
     int size = e.size();
     if (size == 1) {
         return;
     }
     if (abs(direction) != 1) {
         throw std::logic_error(
             "Invalid direction, only 1 and -1 are valid values.");
     }
 
     if ((e.cols() == 1 || e.rows() == 1) && size > 1) {
         auto start_ptr = direction == 1 ? e.data() + size - 2 : e.data() + 1;
         auto end_ptr = direction == 1 ? e.data() : e.data() + size - 1;
         while (start_ptr != end_ptr) {
             *(start_ptr + direction) = *start_ptr;
             start_ptr -= direction;
         }
         *(start_ptr + direction) = *start_ptr;
     } else {
         auto start_ptr = direction == 1 ? e.data() + size - 2 * e.rows()
                                         : e.data() + e.rows();
         auto end_ptr = direction == 1 ? e.data() : e.data() + size - e.rows();
         auto span = e.rows();
         while (start_ptr != end_ptr) {
             std::copy(start_ptr, start_ptr + span,
                       start_ptr + direction * span);
             start_ptr -= (direction * span);
         }
         std::copy(start_ptr, start_ptr + span, start_ptr + direction * span);
     }
 }
 
 /**
  * @brief Normalize the yaws between points on the basis of final yaw angle
  *    of the trajectory.
  * @param last_yaws Final yaw angles of the trajectories.
  * @param yaw_between_points Yaw angles calculated between x and y coordinates
  * of the trajectories.
  * @return Normalized yaw between points.
  */
 inline auto normalize_yaws_between_points(
     const Eigen::Ref<const Eigen::ArrayXf>& last_yaws,
     const Eigen::Ref<const Eigen::ArrayXf>& yaw_between_points) {
     Eigen::ArrayXf yaws =
         shortest_angular_distance(last_yaws, yaw_between_points).abs();
     int size = yaws.size();
     Eigen::ArrayXf yaws_between_points_corrected(size);
     for (int i = 0; i != size; i++) {
         const float& yaw_between_point = yaw_between_points[i];
         yaws_between_points_corrected[i] =
             yaws[i] < M_PIF_2 ? yaw_between_point
                               : normalize_angle(yaw_between_point + M_PIF);
     }
     return yaws_between_points_corrected;
 }
 
 /**
  * @brief Normalize the yaws between points on the basis of goal angle.
  * @param goal_yaw Goal yaw angle.
  * @param yaw_between_points Yaw angles calculated between x and y coordinates
  * of the trajectories.
  * @return Normalized yaw between points
  */
 inline auto normalize_yaws_between_points(
     const float goal_yaw,
     const Eigen::Ref<const Eigen::ArrayXf>& yaw_between_points) {
     int size = yaw_between_points.size();
     Eigen::ArrayXf yaws_between_points_corrected(size);
     for (int i = 0; i != size; i++) {
         const float& yaw_between_point = yaw_between_points[i];
         yaws_between_points_corrected[i] =
             fabs(normalize_angle(yaw_between_point - goal_yaw)) < M_PIF_2
                 ? yaw_between_point
                 : normalize_angle(yaw_between_point + M_PIF);
     }
     return yaws_between_points_corrected;
 }
 
 /**
  * @brief Clamps the input between the given lower and upper bounds.
  * @param lower_bound Lower bound.
  * @param upper_bound Upper bound.
  * @return Clamped output.
  */
 inline float clamp(const float lower_bound, const float upper_bound,
                    const float input) {
     return std::min(upper_bound, std::max(input, lower_bound));
 }

 /**
  * @brief Clamp a desired velocity to respect acceleration limits from last_vel.
  */
 inline float clampVelocityByAccel(const float last_vel, const float curr_vel,
                                 const float min_delta,
                                 const float max_delta) {
     if (last_vel >= 0.0f) {
         return clamp(last_vel + min_delta, last_vel + max_delta, curr_vel);
     }
     return clamp(last_vel - max_delta, last_vel - min_delta, curr_vel);
 }
 
 }  // namespace tools
 }  // namespace mppi_controller
 }  // namespace controller
 }  // namespace control
 }  // namespace autonomy