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

#include <cassert>
#include <cmath>
#include <complex>
#include <iterator>
#include <optional>

#include "autonomy/control/controller/teb_controller/geometry/obstacle.hpp"
#include "autonomy/control/controller/teb_controller/pose2d_utils.hpp"

namespace autonomy {
namespace control {
namespace teb_controller {

// Pose samples along the spatial part of the band.
using PoseSequence = std::vector<Pose2D>;

// Segment durations along the temporal part of the band.
using TimeDiffSequence = std::vector<double>;

/**
 * @brief Timed elastic band trajectory representation
 *
 * Stores a sequence of pose samples and the time intervals between consecutive
 * poses. Used as the optimization variable for TEB planning.
 */
class TimedElasticBand
{
public:
    /**
     * Define TimedElasticBand::SharedPtr type
     */
    AUTONOMY_SHARED_PTR_DEFINITIONS(TimedElasticBand);

    /**
     * @brief Destructor for TimedElasticBand
     */
    virtual ~TimedElasticBand();

    /**
     * @brief Returns the mutable pose sequence
     */
    PoseSequence& Poses() {
        return pose_vec_;
    };

    /**
     * @brief Returns the pose sequence
     */
    const PoseSequence& Poses() const {
        return pose_vec_;
    };

    /**
     * @brief Returns the mutable time-diff sequence
     */
    TimeDiffSequence& Timediffs() {
        return timediff_vec_;
    };

    /**
     * @brief Returns the time-diff sequence
     */
    const TimeDiffSequence& Timediffs() const {
        return timediff_vec_;
    };

    /**
     * @brief Returns the time difference at the given index
     *
     * @param index Index into the time-diff sequence
     * @return Reference to the time difference value
     */
    double& TimeDiff(int index) {
        assert(index < SizeTimeDiffs());
        return timediff_vec_.at(index);
    }

    /**
     * @brief Returns the time difference at the given index
     *
     * @param index Index into the time-diff sequence
     * @return Const reference to the time difference value
     */
    const double& TimeDiff(int index) const {
        assert(index < SizeTimeDiffs());
        return timediff_vec_.at(index);
    }

    /**
     * @brief Returns the pose at the given index
     *
     * @param index Index into the pose sequence
     * @return Reference to the pose sample
     */
    Pose2D& Pose(int index) {
        assert(index < SizePoses());
        return pose_vec_.at(index);
    }

    /**
     * @brief Returns the pose at the given index
     *
     * @param index Index into the pose sequence
     * @return Const reference to the pose sample
     */
    const Pose2D& Pose(int index) const {
        assert(index < SizePoses());
        return pose_vec_.at(index);
    }

    /**
     * @brief Returns the last pose in the band
     */
    Pose2D& BackPose() {
        return pose_vec_.back();
    }

    /**
     * @brief Returns the last pose in the band
     */
    const Pose2D& BackPose() const {
        return pose_vec_.back();
    }

    /**
     * @brief Returns the last time difference in the band
     */
    double& BackTimeDiff() {
        return timediff_vec_.back();
    }

    /**
     * @brief Returns the last time difference in the band
     */
    const double& BackTimeDiff() const {
        return timediff_vec_.back();
    }

    /**
     * @brief Appends a pose sample to the band
     *
     * @param pose Pose to append
     * @param fixed Whether the pose is fixed during optimization
     */
    void AddPose(const Pose2D& pose, bool fixed = false);

    /**
     * @brief Appends a pose sample to the band
     *
     * @param position Position of the pose
     * @param theta Heading of the pose
     * @param fixed Whether the pose is fixed during optimization
     */
    void AddPose(const Point& position, double theta, bool fixed = false);

    /**
     * @brief Appends a pose sample to the band
     *
     * @param x X coordinate of the pose
     * @param y Y coordinate of the pose
     * @param theta Heading of the pose
     * @param fixed Whether the pose is fixed during optimization
     */
    void AddPose(double x, double y, double theta, bool fixed = false);

    /**
     * @brief Appends a time difference to the band
     *
     * @param dt Time interval between the last two poses
     * @param fixed Whether the time difference is fixed during optimization
     */
    void AddTimeDiff(double dt, bool fixed = false);

    /**
     * @brief Appends a pose and its preceding time difference
     *
     * Requires that the band already contains one more pose than time diff.
     *
     * @param pose Pose to append
     * @param dt Time interval between the previous pose and this pose
     */
    void AddPoseAndTimeDiff(const Pose2D& pose, double dt);

    /**
     * @brief Appends a pose and its preceding time difference
     *
     * @param position Position of the pose
     * @param theta Heading of the pose
     * @param dt Time interval between the previous pose and this pose
     */
    void AddPoseAndTimeDiff(const Point& position, double theta, double dt);

    /**
     * @brief Appends a pose and its preceding time difference
     *
     * @param x X coordinate of the pose
     * @param y Y coordinate of the pose
     * @param theta Heading of the pose
     * @param dt Time interval between the previous pose and this pose
     */
    void AddPoseAndTimeDiff(double x, double y, double theta, double dt);

    /**
     * @brief Inserts a pose at the given index
     *
     * @param index Insertion index
     * @param pose Pose to insert
     */
    void InsertPose(int index, const Pose2D& pose);

    /**
     * @brief Inserts a pose at the given index
     *
     * @param index Insertion index
     * @param position Position of the pose
     * @param theta Heading of the pose
     */
    void InsertPose(int index, const Point& position, double theta);

    /**
     * @brief Inserts a pose at the given index
     *
     * @param index Insertion index
     * @param x X coordinate of the pose
     * @param y Y coordinate of the pose
     * @param theta Heading of the pose
     */
    void InsertPose(int index, double x, double y, double theta);

    /**
     * @brief Inserts a time difference at the given index
     *
     * @param index Insertion index
     * @param dt Time interval value to insert
     */
    void InsertTimeDiff(int index, double dt);

    /**
     * @brief Removes the pose at the given index
     *
     * @param index Index of the pose to remove
     */
    void DeletePose(int index);

    /**
     * @brief Removes a range of poses starting at the given index
     *
     * @param index Start index of the range to remove
     * @param number Number of poses to remove
     */
    void DeletePoses(int index, int number);

    /**
     * @brief Removes the time difference at the given index
     *
     * @param index Index of the time difference to remove
     */
    void DeleteTimeDiff(int index);

    /**
     * @brief Removes a range of time differences starting at the given index
     *
     * @param index Start index of the range to remove
     * @param number Number of time differences to remove
     */
    void DeleteTimeDiffs(int index, int number);

    /**
     * @brief Initializes the band from start and goal with uniform spacing
     *
     * Inserts intermediate poses along the straight line between start and goal
     * when diststep is non-zero. Fails if the band is already initialized.
     *
     * @param start Start pose
     * @param goal Goal pose
     * @param diststep Distance between intermediate samples; zero disables
     * intermediate insertion
     * @param max_velocity_x Maximum translational velocity used to estimate dt
     * @param min_samples Minimum number of pose samples excluding the goal
     * @param guess_backwards_motion Whether to infer backward motion from start
     * orientation
     * @return true if initialization succeeded, else false
     */
    bool InitTrajectoryToGoal(const Pose2D& start, const Pose2D& goal,
                              double diststep = 0, double max_velocity_x = 0.5,
                              int min_samples = 3,
                              bool guess_backwards_motion = false);

    /**
     * @brief Initializes the band from an arbitrary path iterator
     *
     * Samples poses along the path and estimates time differences from
     * velocity and optional acceleration limits. Fails if the band is already
     * initialized.
     *
     * @param path_start Iterator to the first path element
     * @param path_end Iterator past the last path element
     * @param fun_position Callable that extracts a 2D position from a path element
     * @param max_velocity_x Maximum translational velocity
     * @param max_angular_velocity Maximum rotational velocity
     * @param max_acc_x Optional maximum translational acceleration
     * @param max_acc_theta Optional maximum rotational acceleration
     * @param start_orientation Optional fixed start heading
     * @param goal_orientation Optional fixed goal heading
     * @param min_samples Minimum number of pose samples excluding the goal
     * @param guess_backwards_motion Whether to infer backward motion
     * @return true if initialization succeeded, else false
     */
    template <typename BidirIter, typename Fun>
    bool InitTrajectoryToGoal(BidirIter path_start, BidirIter path_end,
                              Fun fun_position, double max_velocity_x,
                              double max_angular_velocity,
                              std::optional<double> max_acc_x,
                              std::optional<double> max_acc_theta,
                              std::optional<double> start_orientation,
                              std::optional<double> goal_orientation,
                              int min_samples = 3,
                              bool guess_backwards_motion = false);

    /**
     * @brief Initializes the band from a stamped global plan
     *
     * Converts the plan to pose samples and estimates segment durations from
     * velocity limits. Fails if the band is already initialized.
     *
     * @param plan Global plan as stamped poses
     * @param max_velocity_x Maximum translational velocity
     * @param max_angular_velocity Maximum rotational velocity
     * @param estimate_orient Whether to estimate intermediate headings from the
     * plan geometry
     * @param min_samples Minimum number of pose samples excluding the goal
     * @param guess_backwards_motion Whether to infer backward motion
     * @return true if initialization succeeded, else false
     */
    bool InitTrajectoryToGoal(
        const std::vector<autonomy::commsgs::geometry_msgs::PoseStamped>& plan,
        double max_velocity_x, double max_angular_velocity,
        bool estimate_orient = false, int min_samples = 3,
        bool guess_backwards_motion = false);

    /**
     * @brief Updates start and/or goal and prunes passed trajectory states
     *
     * Replaces the first pose with new_start when provided and removes poses
     * already passed by the robot. Replaces the last pose with new_goal when
     * provided.
     *
     * @param new_start Optional new start pose
     * @param new_goal Optional new goal pose
     * @param min_samples Minimum number of remaining pose samples after pruning
     */
    void UpdateAndPruneTEB(const Pose2D* new_start, const Pose2D* new_goal,
                           int min_samples = 3);

    /**
     * @brief Resizes the band by inserting or removing pose/time-diff pairs
     *
     * Adjusts the temporal resolution so that time differences stay near
     * reference_time_step, subject to hysteresis and sample count limits.
     *
     * @param reference_time_step Target time difference between consecutive poses
     * @param time_step_hysteresis Hysteresis band around the reference step
     * @param min_samples Minimum number of time differences to retain
     * @param max_samples Maximum number of time differences allowed
     * @param fast_mode If true, perform at most one resize pass
     */
    void AutoResize(double reference_time_step, double time_step_hysteresis,
                    int min_samples = 3, int max_samples = 1000,
                    bool fast_mode = false);

    /**
     * @brief Sets whether a pose sample is fixed during optimization
     *
     * @param index Index of the pose sample
     * @param status True to fix the pose, false to leave it free
     */
    void SetPoseVertexFixed(int index, bool status);

    /**
     * @brief Sets whether a time difference is fixed during optimization
     *
     * @param index Index of the time difference
     * @param status True to fix the time difference, false to leave it free
     */
    void SetTimeDiffVertexFixed(int index, bool status);

    /**
     * @brief Returns whether a pose sample is fixed during optimization
     *
     * @param index Index of the pose sample
     * @return true if the pose is fixed, else false
     */
    bool IsPoseVertexFixed(int index) const;

    /**
     * @brief Returns whether a time difference is fixed during optimization
     *
     * @param index Index of the time difference
     * @return true if the time difference is fixed, else false
     */
    bool IsTimeDiffVertexFixed(int index) const;

    /**
     * @brief Clears all pose and time-diff samples
     */
    void ClearTimedElasticBand();

    /**
     * @brief Finds the closest trajectory pose to a reference point
     *
     * @param ref_point Reference point in the plane
     * @param distance Optional output for the minimum distance
     * @param begin_idx First pose index to consider
     * @return Index of the closest pose, or -1 if begin_idx is invalid
     */
    int FindClosestTrajectoryPose(const Point& ref_point,
                                  double* distance = nullptr,
                                  int begin_idx = 0) const;

    /**
     * @brief Finds the closest trajectory pose to a reference line segment
     *
     * @param ref_line_start Start point of the reference segment
     * @param ref_line_end End point of the reference segment
     * @param distance Optional output for the minimum distance
     * @return Index of the closest pose, or -1 if the band is empty
     */
    int FindClosestTrajectoryPose(const Point& ref_line_start,
                                  const Point& ref_line_end,
                                  double* distance = nullptr) const;

    /**
     * @brief Finds the closest trajectory pose to a polygon
     *
     * @param vertices Vertices of the reference polygon
     * @param distance Optional output for the minimum distance
     * @return Index of the closest pose
     */
    int FindClosestTrajectoryPose(const Point2dContainer& vertices,
                                  double* distance = nullptr) const;

    /**
     * @brief Finds the closest trajectory pose to an obstacle
     *
     * Dispatches to the appropriate distance query based on obstacle type.
     *
     * @param obstacle Obstacle to measure against
     * @param distance Optional output for the minimum distance
     * @return Index of the closest pose
     */
    int FindClosestTrajectoryPose(const Obstacle& obstacle,
                                  double* distance = nullptr) const;

    /**
     * @brief Returns the number of pose samples in the band
     */
    int SizePoses() const {
        return (int)pose_vec_.size();
    };

    /**
     * @brief Returns the number of time differences in the band
     */
    int SizeTimeDiffs() const {
        return (int)timediff_vec_.size();
    };

    /**
     * @brief Returns whether the band contains pose and time-diff samples
     *
     * @return true if both sequences are non-empty, else false
     */
    bool IsInit() const {
        return !timediff_vec_.empty() && !pose_vec_.empty();
    }

    /**
     * @brief Returns the sum of all time differences in the band
     */
    double GetSumOfAllTimeDiffs() const;

    /**
     * @brief Returns the sum of time differences up to but excluding index
     *
     * @param index Exclusive upper index into the time-diff sequence
     * @return Accumulated time up to the given index
     */
    double GetSumOfTimeDiffsUpToIdx(int index) const;

    /**
     * @brief Returns the accumulated spatial length of the trajectory
     */
    double GetAccumulatedDistance() const;

    /**
     * @brief Checks whether the trajectory stays inside a region around the robot
     *
     * Verifies that poses lie within radius of the start pose. Optionally applies
     * a separate distance limit behind the robot heading.
     *
     * @param radius Maximum allowed distance from the start pose
     * @param max_dist_behind_robot Maximum allowed distance behind the robot;
     * negative values disable the rear check
     * @param skip_poses Number of intermediate poses to skip between checks
     * @return true if all checked poses are inside the region, else false
     */
    bool IsTrajectoryInsideRegion(double radius,
                                  double max_dist_behind_robot = -1,
                                  int skip_poses = 0);

protected:
    PoseSequence pose_vec_;
    std::vector<bool> pose_fixed_;
    TimeDiffSequence timediff_vec_;
    std::vector<bool> timediff_fixed_;
};

template <typename BidirIter, typename Fun>
bool TimedElasticBand::InitTrajectoryToGoal(
    BidirIter path_start, BidirIter path_end, Fun fun_position,
    double max_velocity_x, double /*max_angular_velocity*/,
    std::optional<double> max_acc_x, std::optional<double> /*max_acc_theta*/,
    std::optional<double> start_orientation,
    std::optional<double> goal_orientation, int min_samples,
    bool guess_backwards_motion) {
    constexpr double kPi = 3.14159265358979323846;
    const Point start_position = fun_position(*path_start);
    const Point goal_position = fun_position(*std::prev(path_end));

    // Infer initial heading and optional backward motion.
    bool backwards = false;
    double start_orient = 0.0;
    double goal_orient = 0.0;
    if (start_orientation.has_value()) {
        start_orient = *start_orientation;
        if (guess_backwards_motion &&
            Dot(goal_position - start_position,
                MakePoint(std::cos(start_orient), std::sin(start_orient))) < 0) {
            backwards = true;
        }
    } else {
        const Point start_to_goal = goal_position - start_position;
        start_orient = std::atan2(start_to_goal.y, start_to_goal.x);
    }

    double timestep = 1.0;
    goal_orient =
        goal_orientation.has_value() ? *goal_orientation : start_orient;

    if (IsInit()) {
        ADEBUG
            << "InitTrajectoryToGoal skipped: trajectory already initialized.";
        ADEBUG << "Current sizes: poses=" << SizePoses()
               << ", time_diffs=" << SizeTimeDiffs();
        return false;
    }

    AddPose(start_position, start_orient, true);
    std::advance(path_start, 1);
    std::advance(path_end, -1);
    int idx = 0;
    // Add interior samples and estimate per-segment dt.
    for (; path_start != path_end; ++path_start) {
        const Point curr_point = fun_position(*path_start);
        const Point diff_last = curr_point - Position(Pose(idx));
        const double diff_norm = Norm(diff_last);

        const double timestep_vel = diff_norm / max_velocity_x;
        if (max_acc_x.has_value()) {
            const double timestep_acc = std::sqrt(2.0 * diff_norm / *max_acc_x);
            timestep =
                (timestep_vel < timestep_acc) ? timestep_acc : timestep_vel;
        } else {
            timestep = timestep_vel;
        }
        if (timestep <= 0.0) {
            timestep = 0.2;
        }

        double yaw = std::atan2(diff_last.y, diff_last.x);
        if (backwards) {
            yaw = autonomy::common::NormalizeAngle(yaw + kPi);
        }
        AddPoseAndTimeDiff(curr_point, yaw, timestep);
        ++idx;
    }

    const Point diff = goal_position - Position(Pose(idx));
    const double diff_norm = Norm(diff);
    const double timestep_vel = diff_norm / max_velocity_x;
    if (max_acc_x.has_value()) {
        const double timestep_acc = std::sqrt(2.0 * diff_norm / *max_acc_x);
        timestep = (timestep_vel < timestep_acc) ? timestep_acc : timestep_vel;
    } else {
        timestep = timestep_vel;
    }

    const Pose2D goal{goal_position.x, goal_position.y, goal_orient};
    if (SizePoses() < min_samples - 1) {
        ADEBUG << "Generated samples below min_samples=" << min_samples
               << "; inserting extra samples.";
        while (SizePoses() < min_samples - 1) {
            timestep /= 2.0;
            AddPoseAndTimeDiff(AveragePose2D(BackPose(), goal), timestep);
        }
    }
    AddPoseAndTimeDiff(goal, timestep);
    SetPoseVertexFixed(SizePoses() - 1, true);
    return true;
}

}  // namespace teb_controller
}  // namespace control
}  // namespace autonomy
