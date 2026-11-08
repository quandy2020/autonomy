/*
 * Copyright 2026 The Navigation Authors (navigation)
 */

/**
 * @file twist2d.hpp
 * @brief Planar velocity alias over automsgs geometry_msgs::Twist2D.
 */

#pragma once

#include <cmath>

#include "automsgs/msgs/geometry_msgs/twist.pb.h"

namespace navigation
{
namespace common
{

/** @brief Planar velocity (vx, vy, omega) backed by the automsgs message. */
using Twist2D = automsgs::msgs::geometry_msgs::Twist2D;

/**
 * @brief Build a planar velocity.
 *
 * @param[in] linear_x Forward velocity in m/s.
 * @param[in] linear_y Lateral velocity in m/s.
 * @param[in] angular_z Yaw rate in rad/s.
 */
inline Twist2D MakeTwist2D(double linear_x, double linear_y, double angular_z)
{
    Twist2D twist;
    twist.set_x(static_cast<float>(linear_x));
    twist.set_y(static_cast<float>(linear_y));
    twist.set_theta(static_cast<float>(angular_z));
    return twist;
}

/**
 * @brief Translational speed magnitude.
 *
 * @param[in] twist Planar velocity.
 * @return sqrt(vx^2 + vy^2) in metres per second.
 */
inline double Speed(const Twist2D& twist)
{
    return std::hypot(static_cast<double>(twist.x()),
                      static_cast<double>(twist.y()));
}

/**
 * @brief Return true when every velocity component is finite.
 *
 * @param[in] twist Velocity to check.
 */
inline bool IsFinite(const Twist2D& twist)
{
    return std::isfinite(twist.x()) && std::isfinite(twist.y()) &&
           std::isfinite(twist.theta());
}

}  // namespace common
}  // namespace navigation
