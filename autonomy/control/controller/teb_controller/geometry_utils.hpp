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

#include <cmath>
#include <vector>

#include "autonomy/commsgs/geometry_msgs.hpp"

namespace autonomy {
namespace control {
namespace teb_controller {

using Pose2D = autonomy::commsgs::geometry_msgs::Pose2D;
using Point = autonomy::commsgs::geometry_msgs::Point;
using Vector3 = autonomy::commsgs::geometry_msgs::Vector3;
using Twist2D = autonomy::commsgs::geometry_msgs::Twist2D;

using PointContainer = std::vector<Point>;

inline Point MakePoint(double x, double y, double z = 0.0) {
    return Point{x, y, z};
}

inline bool PointsApproxEqual(const Point& a, const Point& b,
                              double epsilon = 1e-9) {
    return std::abs(a.x - b.x) < epsilon && std::abs(a.y - b.y) < epsilon &&
           std::abs(a.z - b.z) < epsilon;
}

inline void SetZero(Point& point) {
    point.x = 0.0;
    point.y = 0.0;
    point.z = 0.0;
}

inline void SetZero(Vector3& vector) {
    vector.x = 0.0f;
    vector.y = 0.0f;
    vector.z = 0.0f;
}

inline Point operator+(const Point& a, const Point& b) {
    return MakePoint(a.x + b.x, a.y + b.y, a.z + b.z);
}

inline Point operator-(const Point& a, const Point& b) {
    return MakePoint(a.x - b.x, a.y - b.y, a.z - b.z);
}

inline Point operator*(const Point& point, double scale) {
    return MakePoint(point.x * scale, point.y * scale, point.z * scale);
}

inline Point& operator+=(Point& a, const Point& b) {
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
    return a;
}

inline double Dot(const Point& a, const Point& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline double Cross2d(const Point& a, const Point& b) {
    return a.x * b.y - b.x * a.y;
}

inline Point Rotate2D(const Point& point, double angle_rad) {
    const double cos_angle = std::cos(angle_rad);
    const double sin_angle = std::sin(angle_rad);
    return MakePoint(cos_angle * point.x - sin_angle * point.y,
                     sin_angle * point.x + cos_angle * point.y);
}

inline double Norm(const Point& point) {
    return std::sqrt(Dot(point, point));
}

inline double SquaredNorm(const Point& point) {
    return Dot(point, point);
}

inline Point Normalized(const Point& point) {
    const double n = Norm(point);
    if (n < 1e-15) {
        return MakePoint();
    }
    return point * (1.0 / n);
}

inline Vector3 operator+(const Vector3& a, const Vector3& b) {
    return Vector3{a.x + b.x, a.y + b.y, a.z + b.z};
}

inline Vector3 operator-(const Vector3& a, const Vector3& b) {
    return Vector3{a.x - b.x, a.y - b.y, a.z - b.z};
}

inline Vector3 operator*(const Vector3& vector, double scale) {
    return Vector3{static_cast<float>(vector.x * scale),
                   static_cast<float>(vector.y * scale),
                   static_cast<float>(vector.z * scale)};
}

inline Vector3 operator*(const Vector3& vector, float scale) {
    return Vector3{vector.x * scale, vector.y * scale, vector.z * scale};
}

inline Vector3 operator*(float scale, const Vector3& vector) {
    return vector * scale;
}

inline Vector3& operator+=(Vector3& a, const Vector3& b) {
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
    return a;
}

inline float Dot(const Vector3& a, const Vector3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline float Norm(const Vector3& vector) {
    return std::sqrt(Dot(vector, vector));
}

inline float SquaredNorm(const Vector3& vector) {
    return Dot(vector, vector);
}

inline Vector3 Cross(const Vector3& a, const Vector3& b) {
    return Vector3{a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
                   a.x * b.y - a.y * b.x};
}

inline Point PointFromPose2D(const Pose2D& pose) {
    return MakePoint(pose.x, pose.y);
}

inline Pose2D Pose2DFromPoint(const Point& point, double theta = 0.0) {
    return Pose2D{point.x, point.y, theta};
}

inline Point VelocityDisplacement(const Twist2D& velocity, double t) {
    return MakePoint(velocity.x * t, velocity.y * t);
}

inline Point PredictPosition(const Point& position, const Twist2D& velocity,
                             double t) {
    return position + VelocityDisplacement(velocity, t);
}

}  // namespace teb_controller
}  // namespace control
}  // namespace autonomy
