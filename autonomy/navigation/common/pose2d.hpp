/*
 * Copyright 2026 The Navigation Authors (navigation)
 */

/**
 * @file pose2d.hpp
 * @brief Planar pose alias over automsgs geometry_msgs::Pose2D.
 */

#pragma once

#include <cmath>

#include "automsgs/msgs/geometry_msgs/pose2d.pb.h"

namespace navigation
{
namespace common
{

/** @brief Planar pose (x, y, theta) backed by the automsgs message. */
using Pose2D = automsgs::msgs::geometry_msgs::Pose2D;

/**
 * @brief Build a planar pose.
 *
 * @param[in] x X position in metres.
 * @param[in] y Y position in metres.
 * @param[in] theta_rad Heading in radians.
 */
inline Pose2D MakePose2D(double x, double y, double theta_rad)
{
    Pose2D pose;
    pose.set_x(x);
    pose.set_y(y);
    pose.set_theta(theta_rad);
    return pose;
}

/**
 * @brief Compose two planar transforms.
 *
 * @param[in] parent_from_child Transform from child to parent frame.
 * @param[in] child_from_leaf Transform from leaf to child frame.
 * @return Transform from leaf to parent frame.
 */
inline Pose2D Compose(const Pose2D& parent_from_child,
                      const Pose2D& child_from_leaf)
{
    const double c = std::cos(parent_from_child.theta());
    const double s = std::sin(parent_from_child.theta());
    return MakePose2D(parent_from_child.x() + c * child_from_leaf.x() -
                          s * child_from_leaf.y(),
                      parent_from_child.y() + s * child_from_leaf.x() +
                          c * child_from_leaf.y(),
                      parent_from_child.theta() + child_from_leaf.theta());
}

/**
 * @brief Invert a planar transform.
 *
 * @param[in] pose Transform to invert.
 * @return Inverse transform.
 */
inline Pose2D Inverse(const Pose2D& pose)
{
    const double c = std::cos(pose.theta());
    const double s = std::sin(pose.theta());
    return MakePose2D(-(c * pose.x() + s * pose.y()),
                      -(-s * pose.x() + c * pose.y()), -pose.theta());
}

/**
 * @brief Euclidean distance between two planar positions.
 *
 * @param[in] lhs First pose.
 * @param[in] rhs Second pose.
 * @return Distance in metres.
 */
inline double Distance(const Pose2D& lhs, const Pose2D& rhs)
{
    const double dx = lhs.x() - rhs.x();
    const double dy = lhs.y() - rhs.y();
    return std::sqrt(dx * dx + dy * dy);
}

/**
 * @brief Return true when every pose component is finite.
 *
 * @param[in] pose Pose to check.
 */
inline bool IsFinite(const Pose2D& pose)
{
    return std::isfinite(pose.x()) && std::isfinite(pose.y()) &&
           std::isfinite(pose.theta());
}

}  // namespace common
}  // namespace navigation
