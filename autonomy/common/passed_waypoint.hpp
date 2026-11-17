/*
 * Copyright 2026 The Openbot Authors
 *
 * Geometric tests for dropping via-points the robot has already passed so
 * through-poses replans do not route backward.
 */

#pragma once

#include <cmath>

#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>

namespace autonomy {
namespace common {

inline bool WaypointWithinRadius(
    const ::automsgs::msgs::geometry_msgs::PoseStamped& pose, double x,
    double y, double radius)
{
    const double dx = pose.pose().position().x() - x;
    const double dy = pose.pose().position().y() - y;
    return dx * dx + dy * dy <= radius * radius;
}

// True if (x, y) has moved past `from` toward `to` by more than along_m.
// Coincident from/to counts as passed so duplicate vias collapse.
inline bool WaypointPassedToward(
    const ::automsgs::msgs::geometry_msgs::PoseStamped& from,
    const ::automsgs::msgs::geometry_msgs::PoseStamped& to, double x, double y,
    double along_m)
{
    const double ax = from.pose().position().x();
    const double ay = from.pose().position().y();
    const double vx = to.pose().position().x() - ax;
    const double vy = to.pose().position().y() - ay;
    const double len2 = vx * vx + vy * vy;
    if (len2 < 1e-8) {
        return true;
    }
    const double along = ((x - ax) * vx + (y - ay) * vy) / std::sqrt(len2);
    return along > along_m;
}

}  // namespace common
}  // namespace autonomy
