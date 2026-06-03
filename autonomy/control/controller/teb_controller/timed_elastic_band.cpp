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

#include "autonomy/control/controller/teb_controller/timed_elastic_band.hpp"

#include "autonomy/control/controller/teb_controller/geometry/line_obstacle.hpp"
#include "autonomy/control/controller/teb_controller/geometry/point_obstacle.hpp"
#include "autonomy/control/controller/teb_controller/geometry/polygon_obstacle.hpp"

namespace autonomy {
namespace control {
namespace teb_controller {

namespace {
/**
 * estimate the time to move from start to end.
 * Assumes constant velocity for the motion.
 */
double estimateDeltaT(const Pose2D& start, const Pose2D& end,
                      double max_velocity_x, double max_angular_velocity) {
    double dt_constant_motion = 0.1;
    if (max_velocity_x > 0) {
        double trans_dist = Norm(Position(end) - Position(start));
        dt_constant_motion = trans_dist / max_velocity_x;
    }
    if (max_angular_velocity > 0) {
        double rot_dist = std::abs(
            autonomy::common::AngleDiff(start.theta, end.theta));
        dt_constant_motion =
            std::max(dt_constant_motion, rot_dist / max_angular_velocity);
    }
    return dt_constant_motion;
}
}  // namespace


TimedElasticBand::~TimedElasticBand() {
    ADEBUG << "Destructor TimedElasticBand...";
    ClearTimedElasticBand();
}

void TimedElasticBand::AddPose(const Pose2D& pose, bool fixed) {
    pose_vec_.push_back(pose);
    pose_fixed_.push_back(fixed);
}

void TimedElasticBand::AddPose(const Point& position, double theta, bool fixed) {
    pose_vec_.push_back(Pose2D{position.x, position.y, theta});
    pose_fixed_.push_back(fixed);
}

void TimedElasticBand::AddPose(double x, double y, double theta, bool fixed) {
    pose_vec_.push_back(Pose2D{x, y, theta});
    pose_fixed_.push_back(fixed);
}

void TimedElasticBand::AddTimeDiff(double dt, bool fixed) {
    assert(dt > 0.0 && "Adding a timediff requires a positive dt");
    timediff_vec_.push_back(dt);
    timediff_fixed_.push_back(fixed);
}

void TimedElasticBand::AddPoseAndTimeDiff(double x, double y, double angle,
                                          double dt) {
    if (SizePoses() != SizeTimeDiffs()) {
        AddPose(x, y, angle, false);
        AddTimeDiff(dt, false);
    } else {
        AERROR << "Method AddPoseAndTimeDiff: Add one single Pose first. Timediff describes the time difference between last conf and given conf";  
    }
}

void TimedElasticBand::AddPoseAndTimeDiff(const Pose2D& pose, double dt) {
    if (SizePoses() != SizeTimeDiffs()) {
        AddPose(pose, false);
        AddTimeDiff(dt, false);
    } else {
        AERROR << "Method AddPoseAndTimeDiff: Add one single Pose first. Timediff describes the time difference between last conf and given conf";  
    }
}

void TimedElasticBand::AddPoseAndTimeDiff(const Point& position, double theta,
                                          double dt) {
    if (SizePoses() != SizeTimeDiffs()) {
        AddPose(position, theta, false);
        AddTimeDiff(dt, false);
    } else {
        AERROR << "Method AddPoseAndTimeDiff: Add one single Pose first. Timediff describes the time difference between last conf and given conf";  
    }
}

void TimedElasticBand::DeletePose(int index) {
    assert(index < pose_vec_.size());
    pose_vec_.erase(pose_vec_.begin() + index);
    pose_fixed_.erase(pose_fixed_.begin() + index);
}

void TimedElasticBand::DeletePoses(int index, int number) {
    assert(index + number <= (int)pose_vec_.size());
    pose_vec_.erase(pose_vec_.begin() + index,
                    pose_vec_.begin() + index + number);
    pose_fixed_.erase(pose_fixed_.begin() + index,
                      pose_fixed_.begin() + index + number);
}

void TimedElasticBand::DeleteTimeDiff(int index) {
    assert(index < (int)timediff_vec_.size());
    timediff_vec_.erase(timediff_vec_.begin() + index);
    timediff_fixed_.erase(timediff_fixed_.begin() + index);
}

void TimedElasticBand::DeleteTimeDiffs(int index, int number) {
    assert(index + number <= timediff_vec_.size());
    timediff_vec_.erase(timediff_vec_.begin() + index,
                        timediff_vec_.begin() + index + number);
    timediff_fixed_.erase(timediff_fixed_.begin() + index,
                          timediff_fixed_.begin() + index + number);
}

void TimedElasticBand::InsertPose(int index, const Pose2D& pose) {
    pose_vec_.insert(pose_vec_.begin() + index, pose);
    pose_fixed_.insert(pose_fixed_.begin() + index, false);
}

void TimedElasticBand::InsertPose(int index, const Point& position,
                                  double theta) {
    pose_vec_.insert(pose_vec_.begin() + index,
                     Pose2D{position.x, position.y, theta});
    pose_fixed_.insert(pose_fixed_.begin() + index, false);
}

void TimedElasticBand::InsertPose(int index, double x, double y, double theta) {
    pose_vec_.insert(pose_vec_.begin() + index, Pose2D{x, y, theta});
    pose_fixed_.insert(pose_fixed_.begin() + index, false);
}

void TimedElasticBand::InsertTimeDiff(int index, double dt) {
    timediff_vec_.insert(timediff_vec_.begin() + index, dt);
    timediff_fixed_.insert(timediff_fixed_.begin() + index, false);
}

void TimedElasticBand::ClearTimedElasticBand() {
    pose_vec_.clear();
    pose_fixed_.clear();
    timediff_vec_.clear();
    timediff_fixed_.clear();
}

void TimedElasticBand::SetPoseVertexFixed(int index, bool status) {
    assert(index < SizePoses());
    pose_fixed_.at(index) = status;
}

void TimedElasticBand::SetTimeDiffVertexFixed(int index, bool status) {
    assert(index < SizeTimeDiffs());
    timediff_fixed_.at(index) = status;
}

bool TimedElasticBand::IsPoseVertexFixed(int index) const {
    assert(index < SizePoses());
    return pose_fixed_.at(index);
}

bool TimedElasticBand::IsTimeDiffVertexFixed(int index) const {
    assert(index < SizeTimeDiffs());
    return timediff_fixed_.at(index);
}

void TimedElasticBand::AutoResize(double reference_time_step, double time_step_hysteresis,
                                  int min_samples, int max_samples,
                                  bool fast_mode) {
    assert(SizeTimeDiffs() == 0 || SizeTimeDiffs() + 1 == SizePoses());
    /// iterate through all TEB states and add/remove states!
    bool modified = true;

    for (int rep = 0; rep < 100 && modified;
         ++rep)  // actually it should be while(), but we want to make sure to
                 // not get stuck in some oscillation, hence max 100 repitions.
    {
        modified = false;

        for (int i = 0; i < SizeTimeDiffs();
             ++i)  // TimeDiff connects Point(i) with Point(i+1)
        {
            if (TimeDiff(i) > reference_time_step + time_step_hysteresis &&
                SizeTimeDiffs() < max_samples) {
                // LOG_DEBUG(runtime::get_logger("teb_local_planner"),
                // "teb_local_planner: AutoResize() inserting new bandpoint
                // i=%u, #TimeDiffs=%lu",i,SizeTimeDiffs());

                double newtime = 0.5 * TimeDiff(i);

                TimeDiff(i) = newtime;
                InsertPose(i + 1, AveragePose2D(Pose(i), Pose(i + 1)));
                InsertTimeDiff(i + 1, newtime);

                modified = true;
            } else if (TimeDiff(i) < reference_time_step - time_step_hysteresis &&
                       SizeTimeDiffs() >
                           min_samples)  // only remove samples if size is
                                         // larger than min_samples.
            {
                // LOG_DEBUG(runtime::get_logger("teb_local_planner"),
                // "teb_local_planner: AutoResize() deleting bandpoint i=%u,
                // #TimeDiffs=%lu",i,SizeTimeDiffs());

                if (i < ((int)SizeTimeDiffs() - 1)) {
                    TimeDiff(i + 1) = TimeDiff(i + 1) + TimeDiff(i);
                    DeleteTimeDiff(i);
                    DeletePose(i + 1);
                } else {  // last motion should be adjusted, shift time to the
                          // interval before
                    TimeDiff(i - 1) += TimeDiff(i);
                    DeleteTimeDiff(i);
                    DeletePose(i);
                }

                modified = true;
            }
        }
        if (fast_mode)
            break;
    }
}

double TimedElasticBand::GetSumOfAllTimeDiffs() const {
    double time = 0;
    for (double dt : timediff_vec_) {
        time += dt;
    }
    return time;
}

double TimedElasticBand::GetSumOfTimeDiffsUpToIdx(int index) const {
    assert(index <= timediff_vec_.size());

    double time = 0;

    for (int i = 0; i < index; ++i) {
        time += timediff_vec_.at(i);
    }

    return time;
}

double TimedElasticBand::GetAccumulatedDistance() const {
    double dist = 0;

    for (int i = 1; i < SizePoses(); ++i) {
        dist += Norm(Position(Pose(i)) - Position(Pose(i - 1)));
    }
    return dist;
}

bool TimedElasticBand::InitTrajectoryToGoal(const Pose2D& start,
                                            const Pose2D& goal,
                                            double diststep, double max_velocity_x,
                                            int min_samples,
                                            bool guess_backwards_motion) {
    if (!IsInit()) {
        AddPose(start);  // add starting point
        SetPoseVertexFixed(
            0, true);  // StartConf is a fixed constraint during optimization

        double timestep = 0.1;

        if (diststep != 0) {
            Point point_to_goal = Position(goal) - Position(start);
            double dir_to_goal = std::atan2(point_to_goal.y, point_to_goal.x);
            double dx = diststep * std::cos(dir_to_goal);
            double dy = diststep * std::sin(dir_to_goal);
            double orient_init = dir_to_goal;
            // check if the goal is behind the start pose (w.r.t. start
            // orientation)
            if (guess_backwards_motion &&
                Dot(point_to_goal, OrientationUnitVec(start)) < 0)
                orient_init = autonomy::common::NormalizeAngle(orient_init + M_PI);
            // TODO: timestep ~ max_velocity_x_backwards for backwards motions

            double dist_to_goal = Norm(point_to_goal);
            double no_steps_d =
                dist_to_goal / std::abs(diststep);  // ignore negative values
            unsigned int no_steps = (unsigned int)std::floor(no_steps_d);

            if (max_velocity_x > 0)
                timestep = diststep / max_velocity_x;

            for (unsigned int i = 1; i <= no_steps;
                 i++)  // start with 1! starting point had index 0
            {
                if (i == no_steps && no_steps_d == (float)no_steps)
                    break;  // if last conf (depending on stepsize) is equal to
                            // goal conf -> leave loop
                AddPoseAndTimeDiff(start.x + i * dx, start.y + i * dy,
                                   orient_init, timestep);
            }
        }

        // if number of samples is not larger than min_samples, insert manually
        if (SizePoses() < min_samples - 1) {
            ADEBUG << "initTEBtoGoal(): number of generated samples is less than specified by min_samples. Forcing the insertion of more samples...";
            while (SizePoses() <
                   min_samples -
                       1)  // subtract goal point that will be added later
            {
                // simple strategy: interpolate between the current pose and the
                // goal
                Pose2D intermediate_pose = AveragePose2D(BackPose(), goal);
                if (max_velocity_x > 0)
                    timestep = Norm(Position(intermediate_pose) -
                                    Position(BackPose())) /
                               max_velocity_x;
                AddPoseAndTimeDiff(
                    intermediate_pose,
                    timestep);  // let the optimier correct the timestep (TODO:
                                // better initialization
            }
        }

        // add goal
        if (max_velocity_x > 0)
            timestep =
                Norm(Position(goal) - Position(BackPose())) / max_velocity_x;
        AddPoseAndTimeDiff(goal, timestep);  // add goal point
        SetPoseVertexFixed(
            SizePoses() - 1,
            true);  // GoalConf is a fixed constraint during optimization
    } else          // size!=0
    {
        AERROR << "Cannot init TEB between given configuration and goal, because TEB vectors are not empty or TEB is already initialized (call this function before adding states yourself)!";
        return false;
    }
    return true;
}

bool TimedElasticBand::InitTrajectoryToGoal(
    const std::vector<autonomy::commsgs::geometry_msgs::PoseStamped>& Plan,
    double max_velocity_x, double max_angular_velocity, bool estimate_orient,
    int min_samples, bool guess_backwards_motion) {
    if (!IsInit()) {
        Pose2D start = Pose2DFromPose(Plan.front().pose);
        Pose2D goal = Pose2DFromPose(Plan.back().pose);

        AddPose(start);  // add starting point with given orientation
        SetPoseVertexFixed(
            0, true);  // StartConf is a fixed constraint during optimization

        bool backwards = false;
        if (guess_backwards_motion &&
            Dot(Position(goal) - Position(start), OrientationUnitVec(start)) < 0)  // check if the goal is behind the start pose (w.r.t. start
                    // orientation)
            backwards = true;
        // TODO: dt ~ max_velocity_x_backwards for backwards motions

        for (int i = 1; i < (int)Plan.size() - 1; ++i) {
            double yaw;
            if (estimate_orient) {
                // get yaw from the orientation of the distance vector between
                // pose_{i+1} and pose_{i}
                double dx =
                    Plan[i + 1].pose.position.x - Plan[i].pose.position.x;
                double dy =
                    Plan[i + 1].pose.position.y - Plan[i].pose.position.y;
                yaw = std::atan2(dy, dx);
                if (backwards)
                    yaw = autonomy::common::NormalizeAngle(yaw + M_PI);
            } else {
                yaw = GetYawFromOrientation(Plan[i].pose.orientation);
            }
            Pose2D intermediate_pose{Plan[i].pose.position.x,
                                     Plan[i].pose.position.y, yaw};
            double dt = estimateDeltaT(BackPose(), intermediate_pose, max_velocity_x,
                                       max_angular_velocity);
            AddPoseAndTimeDiff(intermediate_pose, dt);
        }

        // if number of samples is not larger than min_samples, insert manually
        if (SizePoses() < min_samples - 1) {
            ADEBUG << "initTEBtoGoal(): number of generated samples is less than specified by min_samples. Forcing the insertion of more samples...";
            while (SizePoses() <
                   min_samples -
                       1)  // subtract goal point that will be added later
            {
                // simple strategy: interpolate between the current pose and the
                // goal
                Pose2D intermediate_pose = AveragePose2D(BackPose(), goal);
                double dt = estimateDeltaT(BackPose(), intermediate_pose,
                                           max_velocity_x, max_angular_velocity);
                AddPoseAndTimeDiff(
                    intermediate_pose,
                    dt);  // let the optimier correct the timestep (TODO: better
                          // initialization
            }
        }

        // Now add final state with given orientation
        double dt = estimateDeltaT(BackPose(), goal, max_velocity_x, max_angular_velocity);
        AddPoseAndTimeDiff(goal, dt);
        SetPoseVertexFixed(
            SizePoses() - 1,
            true);  // GoalConf is a fixed constraint during optimization
    } else          // size!=0
    {
        AERROR << "Cannot init TEB between given configuration and goal, because TEB vectors are not empty or TEB is already initialized (call this function before adding states yourself)!";
        return false;
    }

    return true;
}

int TimedElasticBand::FindClosestTrajectoryPose(const Point& ref_point,
                                                double* distance,
                                                int begin_idx) const {
    int n = SizePoses();
    if (begin_idx < 0 || begin_idx >= n)
        return -1;

    double min_dist_sq = std::numeric_limits<double>::max();
    int min_idx = -1;

    for (int i = begin_idx; i < n; i++) {
        double dist_sq = SquaredNorm(ref_point - Position(Pose(i)));
        if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
            min_idx = i;
        }
    }

    if (distance)
        *distance = std::sqrt(min_dist_sq);

    return min_idx;
}

int TimedElasticBand::FindClosestTrajectoryPose(const Point& ref_line_start,
                                                const Point& ref_line_end,
                                                double* distance) const {
    double min_dist = std::numeric_limits<double>::max();
    int min_idx = -1;

    for (int i = 0; i < SizePoses(); i++) {
        const Point point = Position(Pose(i));
        double dist =
            Distance(point, ref_line_start, ref_line_end);
        if (dist < min_dist) {
            min_dist = dist;
            min_idx = i;
        }
    }

    if (distance)
        *distance = min_dist;
    return min_idx;
}

int TimedElasticBand::FindClosestTrajectoryPose(
    const Point2dContainer& vertices, double* distance) const {
    if (vertices.empty())
        return 0;
    else if (vertices.size() == 1)
        return FindClosestTrajectoryPose(vertices.front());
    else if (vertices.size() == 2)
        return FindClosestTrajectoryPose(vertices.front(), vertices.back());

    double min_dist = std::numeric_limits<double>::max();
    int min_idx = -1;

    for (int i = 0; i < SizePoses(); i++) {
        const Point point = Position(Pose(i));
        double dist_to_polygon = std::numeric_limits<double>::max();
        for (int j = 0; j < (int)vertices.size() - 1; ++j) {
            dist_to_polygon = std::min(
                dist_to_polygon,
                Distance(point, vertices[j], vertices[j + 1]));
        }
        dist_to_polygon = std::min(
            dist_to_polygon,
            Distance(point, vertices.back(), vertices.front()));
        if (dist_to_polygon < min_dist) {
            min_dist = dist_to_polygon;
            min_idx = i;
        }
    }

    if (distance)
        *distance = min_dist;

    return min_idx;
}

int TimedElasticBand::FindClosestTrajectoryPose(const Obstacle& obstacle,
                                                double* distance) const {
    const PointObstacle* pobst = dynamic_cast<const PointObstacle*>(&obstacle);
    if (pobst)
        return FindClosestTrajectoryPose(pobst->position(), distance);

    const LineObstacle* lobst = dynamic_cast<const LineObstacle*>(&obstacle);
    if (lobst)
        return FindClosestTrajectoryPose(lobst->start(), lobst->end(),
                                         distance);

    const PolygonObstacle* polyobst =
        dynamic_cast<const PolygonObstacle*>(&obstacle);
    if (polyobst)
        return FindClosestTrajectoryPose(polyobst->vertices(), distance);

    return FindClosestTrajectoryPose(obstacle.GetCentroid(), distance);
}

void TimedElasticBand::UpdateAndPruneTEB(const Pose2D* new_start,
                                         const Pose2D* new_goal,
                                         int min_samples) {
    // first and simple approach: change only start confs (and virtual start
    // conf for inital velocity) TEST if optimizer can handle this "hard"
    // placement

    if (new_start && SizePoses() > 0) {
        // find nearest state (using l2-norm) in order to prune the trajectory
        // (remove already passed states)
        double dist_cache = Norm(Position(*new_start) - Position(Pose(0)));
        double dist;
        int lookahead =
            std::min<int>(SizePoses() - min_samples,
                          10);  // satisfy min_samples, otherwise max 10 samples

        int nearest_idx = 0;
        for (int i = 1; i <= lookahead; ++i) {
            dist = Norm(Position(*new_start) - Position(Pose(i)));
            if (dist < dist_cache) {
                dist_cache = dist;
                nearest_idx = i;
            } else
                break;
        }

        // prune trajectory at the beginning (and extrapolate sequences at the
        // end if the horizon is fixed)
        if (nearest_idx > 0) {
            // nearest_idx is equal to the number of samples to be removed
            // (since it counts from 0 ;-) ) WARNING delete starting at pose 1,
            // and overwrite the original pose(0) with new_start, since Pose(0)
            // is fixed during optimization!
            DeletePoses(1, nearest_idx);  // delete first states such that the
                                          // closest state is the new first one
            DeleteTimeDiffs(
                1, nearest_idx);  // delete corresponding time differences
        }

        // update start
        Pose(0) = *new_start;
    }

    if (new_goal && SizePoses() > 0) {
        BackPose() = *new_goal;
    }
};

bool TimedElasticBand::IsTrajectoryInsideRegion(double radius,
                                                double max_dist_behind_robot,
                                                int skip_poses) {
    if (SizePoses() <= 0)
        return true;

    double radius_sq = radius * radius;
    double max_dist_behind_robot_sq =
        max_dist_behind_robot * max_dist_behind_robot;
    const Point robot_orient = OrientationUnitVec(Pose(0));

    for (int i = 1; i < SizePoses(); i = i + skip_poses + 1) {
        const Point dist_vec = Position(Pose(i)) - Position(Pose(0));
        double dist_sq = SquaredNorm(dist_vec);

        if (dist_sq > radius_sq) {
            AINFO << "outside robot";
            return false;
        }

        // check behind the robot with a different distance, if specified (or
        // >=0)
        if (max_dist_behind_robot >= 0 && Dot(dist_vec, robot_orient) < 0 &&
            dist_sq > max_dist_behind_robot_sq) {
            AINFO << "outside robot behind";
            return false;
        }
    }
    return true;
}

}  // namespace teb_controller
}  // namespace control
}  // namespace autonomy
