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

#include "autonomy/control/controller/teb_controller/footprint.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace autonomy {
namespace control {
namespace teb_controller {

double PointRobotFootprint::CalculateDistance(const Pose2D& current_pose,
                                              const Obstacle* obstacle) const {
    return obstacle->GetMinimumDistance(Position(current_pose));
}

double PointRobotFootprint::EstimateSpatioTemporalDistance(
    const Pose2D& current_pose, const Obstacle* obstacle, double t) const {
    return obstacle->GetMinimumSpatioTemporalDistance(Position(current_pose),
                                                      t);
}

double PointRobotFootprint::GetInscribedRadius() {
    return 0.0;
}

CircularRobotFootprint::CircularRobotFootprint(double radius)
    : radius_(radius) {}

void CircularRobotFootprint::SetRadius(double radius) {
    radius_ = radius;
}

double CircularRobotFootprint::CalculateDistance(
    const Pose2D& current_pose, const Obstacle* obstacle) const {
    return obstacle->GetMinimumDistance(Position(current_pose)) - radius_;
}

double CircularRobotFootprint::EstimateSpatioTemporalDistance(
    const Pose2D& current_pose, const Obstacle* obstacle, double t) const {
    return obstacle->GetMinimumSpatioTemporalDistance(Position(current_pose),
                                                      t) -
           radius_;
}

void CircularRobotFootprint::VisualizeRobot(
    const Pose2D& current_pose,
    std::vector<autonomy::commsgs::visualization_msgs::Marker>& markers,
    const autonomy::commsgs::std_msgs::ColorRGBA& color) const {
    markers.resize(1);
    autonomy::commsgs::visualization_msgs::Marker& marker = markers.back();
    marker.type = autonomy::commsgs::visualization_msgs::Marker::CYLINDER;
    ToPose3D(current_pose,marker.pose);
    marker.Scale.x = marker.Scale.y = 2 * radius_;  // Scale = diameter
    marker.Scale.z = 0.05;
    marker.color = color;
}

double CircularRobotFootprint::GetInscribedRadius() {
    return radius_;
}

TwoCirclesRobotFootprint::TwoCirclesRobotFootprint(double front_offset,
                                                   double front_radius,
                                                   double rear_offset,
                                                   double rear_radius)
    : front_offset_(front_offset),
      front_radius_(front_radius),
      rear_offset_(rear_offset),
      rear_radius_(rear_radius) {}

void TwoCirclesRobotFootprint::SetParameters(double front_offset,
                                             double front_radius,
                                             double rear_offset,
                                             double rear_radius) {
    front_offset_ = front_offset;
    front_radius_ = front_radius;
    rear_offset_ = rear_offset;
    rear_radius_ = rear_radius;
}

double TwoCirclesRobotFootprint::CalculateDistance(
    const Pose2D& current_pose, const Obstacle* obstacle) const {
    const Point dir = OrientationUnitVec(current_pose);
    double dist_front = obstacle->GetMinimumDistance(Position(current_pose) +
                                                     front_offset_ * dir) -
                        front_radius_;
    double dist_rear = obstacle->GetMinimumDistance(Position(current_pose) -
                                                    rear_offset_ * dir) -
                       rear_radius_;
    return std::min(dist_front, dist_rear);
}

double TwoCirclesRobotFootprint::EstimateSpatioTemporalDistance(
    const Pose2D& current_pose, const Obstacle* obstacle, double t) const {
    const Point dir = OrientationUnitVec(current_pose);
    double dist_front = obstacle->GetMinimumSpatioTemporalDistance(
                            Position(current_pose) + front_offset_ * dir, t) -
                        front_radius_;
    double dist_rear = obstacle->GetMinimumSpatioTemporalDistance(
                           Position(current_pose) - rear_offset_ * dir, t) -
                       rear_radius_;
    return std::min(dist_front, dist_rear);
}

void TwoCirclesRobotFootprint::VisualizeRobot(
    const Pose2D& current_pose,
    std::vector<autonomy::commsgs::visualization_msgs::Marker>& markers,
    const autonomy::commsgs::std_msgs::ColorRGBA& color) const {
    const Point dir = OrientationUnitVec(current_pose);
    if (front_radius_ > 0) {
        markers.push_back(autonomy::commsgs::visualization_msgs::Marker());
        autonomy::commsgs::visualization_msgs::Marker& marker1 =
            markers.front();
        marker1.type = autonomy::commsgs::visualization_msgs::Marker::CYLINDER;
        ToPose3D(current_pose,marker1.pose);
        marker1.pose.position.x += front_offset_ * dir.x;
        marker1.pose.position.y += front_offset_ * dir.y;
        marker1.Scale.x = marker1.Scale.y =
            2 * front_radius_;  // Scale = diameter
        marker1.color = color;
    }
    if (rear_radius_ > 0) {
        markers.push_back(autonomy::commsgs::visualization_msgs::Marker());
        autonomy::commsgs::visualization_msgs::Marker& marker2 = markers.back();
        marker2.type = autonomy::commsgs::visualization_msgs::Marker::CYLINDER;
        ToPose3D(current_pose,marker2.pose);
        marker2.pose.position.x -= rear_offset_ * dir.x;
        marker2.pose.position.y -= rear_offset_ * dir.y;
        marker2.Scale.x = marker2.Scale.y =
            2 * rear_radius_;  // Scale = diameter
        marker2.color = color;
    }
}

double TwoCirclesRobotFootprint::GetInscribedRadius() {
    double min_longitudinal =
        std::min(rear_offset_ + rear_radius_, front_offset_ + front_radius_);
    double min_lateral = std::min(rear_radius_, front_radius_);
    return std::min(min_longitudinal, min_lateral);
}

LineRobotFootprint::LineRobotFootprint(
    const autonomy::commsgs::geometry_msgs::Point& line_start,
    const autonomy::commsgs::geometry_msgs::Point& line_end) {
    SetLine(line_start, line_end);
}

LineRobotFootprint::LineRobotFootprint(const Point& line_start,
                                       const Point& line_end) {
    SetLine(line_start, line_end);
}

void LineRobotFootprint::SetLine(
    const autonomy::commsgs::geometry_msgs::Point& line_start,
    const autonomy::commsgs::geometry_msgs::Point& line_end) {
    line_start_ = line_start;
    line_end_ = line_end;
}

void LineRobotFootprint::SetLine(const Point& line_start, const Point& line_end) {
    line_start_ = line_start;
    line_end_ = line_end;
}

double LineRobotFootprint::CalculateDistance(const Pose2D& current_pose,
                                             const Obstacle* obstacle) const {
    Point line_start_world;
    Point line_end_world;
    TransformToWorld(current_pose, line_start_world, line_end_world);
    return obstacle->GetMinimumDistance(line_start_world, line_end_world);
}

double LineRobotFootprint::EstimateSpatioTemporalDistance(
    const Pose2D& current_pose, const Obstacle* obstacle, double t) const {
    Point line_start_world;
    Point line_end_world;
    TransformToWorld(current_pose, line_start_world, line_end_world);
    return obstacle->GetMinimumSpatioTemporalDistance(line_start_world,
                                                      line_end_world, t);
}

void LineRobotFootprint::VisualizeRobot(
    const Pose2D& current_pose,
    std::vector<autonomy::commsgs::visualization_msgs::Marker>& markers,
    const autonomy::commsgs::std_msgs::ColorRGBA& color) const {
    markers.push_back(autonomy::commsgs::visualization_msgs::Marker());
    autonomy::commsgs::visualization_msgs::Marker& marker = markers.front();
    marker.type = autonomy::commsgs::visualization_msgs::Marker::LINE_STRIP;
    ToPose3D(current_pose,marker.pose);

    autonomy::commsgs::geometry_msgs::Point line_start_world;
    line_start_world.x = line_start_.x;
    line_start_world.y = line_start_.y;
    line_start_world.z = 0;
    marker.points.push_back(line_start_world);

    autonomy::commsgs::geometry_msgs::Point line_end_world;
    line_end_world.x = line_end_.x;
    line_end_world.y = line_end_.y;
    line_end_world.z = 0;
    marker.points.push_back(line_end_world);

    marker.Scale.x = 0.05;
    marker.color = color;
}

double LineRobotFootprint::GetInscribedRadius() {
    return 0.0;
}

void LineRobotFootprint::TransformToWorld(const Pose2D& current_pose,
                                          Point& line_start_world,
                                          Point& line_end_world) const {
    const double cos_th = std::cos(current_pose.theta);
    const double sin_th = std::sin(current_pose.theta);
    line_start_world.x =
        current_pose.x + cos_th * line_start_.x - sin_th * line_start_.y;
    line_start_world.y =
        current_pose.y + sin_th * line_start_.x + cos_th * line_start_.y;
    line_end_world.x =
        current_pose.x + cos_th * line_end_.x - sin_th * line_end_.y;
    line_end_world.y =
        current_pose.y + sin_th * line_end_.x + cos_th * line_end_.y;
}

PolygonRobotFootprint::PolygonRobotFootprint(const Point2dContainer& vertices)
    : vertices_(vertices) {}

void PolygonRobotFootprint::SetVertices(const Point2dContainer& vertices) {
    vertices_ = vertices;
}

double PolygonRobotFootprint::CalculateDistance(
    const Pose2D& current_pose, const Obstacle* obstacle) const {
    Point2dContainer polygon_world(vertices_.size());
    TransformToWorld(current_pose, polygon_world);
    return obstacle->GetMinimumDistance(polygon_world);
}

double PolygonRobotFootprint::EstimateSpatioTemporalDistance(
    const Pose2D& current_pose, const Obstacle* obstacle, double t) const {
    Point2dContainer polygon_world(vertices_.size());
    TransformToWorld(current_pose, polygon_world);
    return obstacle->GetMinimumSpatioTemporalDistance(polygon_world, t);
}

void PolygonRobotFootprint::VisualizeRobot(
    const Pose2D& current_pose,
    std::vector<autonomy::commsgs::visualization_msgs::Marker>& markers,
    const autonomy::commsgs::std_msgs::ColorRGBA& color) const {
    if (vertices_.empty())
        return;

    markers.push_back(autonomy::commsgs::visualization_msgs::Marker());
    autonomy::commsgs::visualization_msgs::Marker& marker = markers.front();
    marker.type = autonomy::commsgs::visualization_msgs::Marker::LINE_STRIP;
    ToPose3D(current_pose,marker.pose);

    for (std::size_t i = 0; i < vertices_.size(); ++i) {
        autonomy::commsgs::geometry_msgs::Point point;
        point.x = vertices_[i].x;
        point.y = vertices_[i].y;
        point.z = 0;
        marker.points.push_back(point);
    }

    autonomy::commsgs::geometry_msgs::Point point;
    point.x = vertices_.front().x;
    point.y = vertices_.front().y;
    point.z = 0;
    marker.points.push_back(point);

    marker.Scale.x = 0.025;
    marker.color = color;
}

double PolygonRobotFootprint::GetInscribedRadius() {
    double min_dist = std::numeric_limits<double>::max();
    const Point center = MakePoint();

    if (vertices_.size() <= 2)
        return 0.0;

    for (int i = 0; i < static_cast<int>(vertices_.size()) - 1; ++i) {
        double vertex_dist = Norm(vertices_[i]);
        double edge_dist =
            Distance(center, vertices_[i], vertices_[i + 1]);
        min_dist = std::min(min_dist, std::min(vertex_dist, edge_dist));
    }

    double vertex_dist = Norm(vertices_.back());
    double edge_dist =
        Distance(center, vertices_.back(), vertices_.front());
    return std::min(min_dist, std::min(vertex_dist, edge_dist));
}

void PolygonRobotFootprint::TransformToWorld(
    const Pose2D& current_pose, Point2dContainer& polygon_world) const {
    double cos_th = std::cos(current_pose.theta);
    double sin_th = std::sin(current_pose.theta);
    for (std::size_t i = 0; i < vertices_.size(); ++i) {
        polygon_world[i].x = current_pose.x + cos_th * vertices_[i].x -
                             sin_th * vertices_[i].y;
        polygon_world[i].y = current_pose.y + sin_th * vertices_[i].x +
                             cos_th * vertices_[i].y;
    }
}

}  // namespace teb_controller
}  // namespace control
}  // namespace autonomy
