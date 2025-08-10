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

#include <vector>
#include <string>

#include "autonomy/common/port.hpp"
#include "autonomy/commsgs/std_msgs.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"

namespace autonomy {
namespace commsgs {
namespace trajectory_msgs {

// Each trajectory point specifies either positions[, velocities[, accelerations]]
// or positions[, effort] for the trajectory to be executed.
// All specified values are in the same order as the joint names in JointTrajectory.msg.
struct JointTrajectoryPoint
{

    // Single DOF joint positions for each joint relative to their "0" position.
    // The units depend on the specific joint type: radians for revolute or
    // continuous joints, and meters for prismatic joints.
    std::vector<double> positions;

    // The rate of change in position of each joint. Units are joint type dependent.
    // Radians/second for revolute or continuous joints, and meters/second for
    // prismatic joints.
    std::vector<double> velocities;

    // Rate of change in velocity of each joint. Units are joint type dependent.
    // Radians/second^2 for revolute or continuous joints, and meters/second^2 for
    // prismatic joints.
    std::vector<double> accelerations;

    // The torque or the force to be applied at each joint. For revolute/continuous
    // joints effort denotes a torque in newton-meters. For prismatic joints, effort
    // denotes a force in newtons.
    std::vector<double> effort;

    // Desired time from the trajectory start to arrive at this trajectory point.
    builtin_interfaces::Duration time_from_start;
};

struct MultiDOFJointTrajectoryPoint
{
    // Each multi-dof joint can specify a transform (up to 6 DOF).
    std::vector<geometry_msgs::Transform> transforms;

    // There can be a velocity specified for the origin of the joint.
    std::vector<geometry_msgs::Twist> velocities;

    // There can be an acceleration specified for the origin of the joint.
    std::vector<geometry_msgs::Twist> accelerations;

    // Desired time from the trajectory start to arrive at this trajectory point.
    builtin_interfaces::Duration time_from_start;

};

struct MultiDOFJointTrajectory
{
    // The header is used to specify the coordinate frame and the reference time for the trajectory durations
    std_msgs::Header header;

    // A representation of a multi-dof joint trajectory (each point is a transformation)
    // Each point along the trajectory will include an array of positions/velocities/accelerations
    // that has the same length as the array of joint names, and has the same order of joints as 
    // the joint names array.

    std::vector<std::string> joint_names;
    std::vector<MultiDOFJointTrajectoryPoint> points;
};

struct JointTrajectory
{
    // The header is used to specify the coordinate frame and the reference time for
    // the trajectory durations
    std_msgs::Header header;

    // The names of the active joints in each trajectory point. These names are
    // ordered and must correspond to the values in each trajectory point.
    std::vector<std::string> joint_names;

    // Array of trajectory points, which describe the positions, velocities,
    // accelerations and/or efforts of the joints at each time point.
    std::vector<JointTrajectoryPoint> points;

};

}  // namespace trajectory_msgs
}  // namespace commsgs
}  // namespace autonomy