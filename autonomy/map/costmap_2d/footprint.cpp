/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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

#include "autonomy/map/costmap_2d/footprint.hpp"

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

#include "autonomy/common/logging.hpp"
#include "autonomy/map/costmap_2d/array_parser.hpp"
#include "autonomy/map/costmap_2d/costmap_math.hpp"

namespace autonomy {
namespace map {
namespace costmap_2d {

std::pair<double, double> calculateMinAndMaxDistances(
    const std::vector<automsgs::msgs::geometry_msgs::Point>& footprint) {
    double min_dist = std::numeric_limits<double>::max();
    double max_dist = 0.0;

    if (footprint.size() <= 2) {
        return std::pair<double, double>(min_dist, max_dist);
    }

    for (unsigned int i = 0; i < footprint.size() - 1; ++i) {
        // check the distance from the robot center point to the first vertex
        double vertex_dist =
            distance(0.0, 0.0, footprint[i].x(), footprint[i].y());
        double edge_dist = distanceToLine(
            0.0, 0.0, footprint[i].x(), footprint[i].y(),
            footprint[i + 1].x(), footprint[i + 1].y());
        min_dist = std::min(min_dist, std::min(vertex_dist, edge_dist));
        max_dist = std::max(max_dist, std::max(vertex_dist, edge_dist));
    }

    // we also need to do the last vertex and the first vertex
    double vertex_dist =
        distance(0.0, 0.0, footprint.back().x(), footprint.back().y());
    double edge_dist =
        distanceToLine(0.0, 0.0, footprint.back().x(), footprint.back().y(),
                       footprint.front().x(), footprint.front().y());
    min_dist = std::min(min_dist, std::min(vertex_dist, edge_dist));
    max_dist = std::max(max_dist, std::max(vertex_dist, edge_dist));

    return std::pair<double, double>(min_dist, max_dist);
}

automsgs::msgs::geometry_msgs::Point32 toPoint32(
    automsgs::msgs::geometry_msgs::Point pt) {
    automsgs::msgs::geometry_msgs::Point32 point32;
    point32.set_x(pt.x());
    point32.set_y(pt.y());
    point32.set_z(pt.z());
    return point32;
}

automsgs::msgs::geometry_msgs::Point toPoint(
    automsgs::msgs::geometry_msgs::Point32 pt) {
    automsgs::msgs::geometry_msgs::Point point;
    point.set_x(pt.x());
    point.set_y(pt.y());
    point.set_z(pt.z());
    return point;
}

automsgs::msgs::geometry_msgs::Polygon toPolygon(
    std::vector<automsgs::msgs::geometry_msgs::Point> pts) {
    automsgs::msgs::geometry_msgs::Polygon polygon;
    for (unsigned int i = 0; i < pts.size(); i++) {
        *polygon.mutable_points()->Add() = toPoint32(pts[i]);
    }
    return polygon;
}

std::vector<automsgs::msgs::geometry_msgs::Point> toPointVector(
    std::shared_ptr<automsgs::msgs::geometry_msgs::Polygon> polygon) {
    std::vector<automsgs::msgs::geometry_msgs::Point> pts;
    for (int i = 0; i < polygon->points_size(); i++) {
        pts.push_back(toPoint(polygon->points(i)));
    }
    return pts;
}

void transformFootprint(
    double x, double y, double theta,
    const std::vector<automsgs::msgs::geometry_msgs::Point>& footprint_spec,
    std::vector<automsgs::msgs::geometry_msgs::Point>& oriented_footprint) {
    // build the oriented footprint at a given location
    oriented_footprint.resize(footprint_spec.size());
    double cos_th = cos(theta);
    double sin_th = sin(theta);
    for (unsigned int i = 0; i < footprint_spec.size(); ++i) {
        double new_x = x + (footprint_spec[i].x() * cos_th -
                            footprint_spec[i].y() * sin_th);
        double new_y = y + (footprint_spec[i].x() * sin_th +
                            footprint_spec[i].y() * cos_th);
        automsgs::msgs::geometry_msgs::Point& new_pt = oriented_footprint[i];
        new_pt.set_x(new_x);
        new_pt.set_y(new_y);
    }
}

void transformFootprint(
    double x, double y, double theta,
    const std::vector<automsgs::msgs::geometry_msgs::Point>& footprint_spec,
    automsgs::msgs::geometry_msgs::PolygonStamped& oriented_footprint) {
    // build the oriented footprint at a given location
    oriented_footprint.mutable_polygon()->clear_points();
    double cos_th = cos(theta);
    double sin_th = sin(theta);
    for (unsigned int i = 0; i < footprint_spec.size(); ++i) {
        automsgs::msgs::geometry_msgs::Point32 new_pt;
        new_pt.set_x(x + (footprint_spec[i].x() * cos_th -
                          footprint_spec[i].y() * sin_th));
        new_pt.set_y(y + (footprint_spec[i].x() * sin_th +
                          footprint_spec[i].y() * cos_th));
        *oriented_footprint.mutable_polygon()->mutable_points()->Add() =
            new_pt;
    }
}

void padFootprint(std::vector<automsgs::msgs::geometry_msgs::Point>& footprint,
                  double padding) {
    // pad footprint in place
    for (unsigned int i = 0; i < footprint.size(); i++) {
        automsgs::msgs::geometry_msgs::Point& pt = footprint[i];
        pt.set_x(pt.x() + sign0(pt.x()) * padding);
        pt.set_y(pt.y() + sign0(pt.y()) * padding);
    }
}

std::vector<automsgs::msgs::geometry_msgs::Point> makeFootprintFromRadius(
    double radius) {
    std::vector<automsgs::msgs::geometry_msgs::Point> points;

    // Loop over 16 angles around a circle making a point each time
    int N = 16;
    automsgs::msgs::geometry_msgs::Point pt;
    for (int i = 0; i < N; ++i) {
        double angle = i * 2 * M_PI / N;
        pt.set_x(cos(angle) * radius);
        pt.set_y(sin(angle) * radius);

        points.push_back(pt);
    }

    return points;
}

bool makeFootprintFromString(
    const std::string& footprint_string,
    std::vector<automsgs::msgs::geometry_msgs::Point>& footprint) {
    std::string error;
    std::vector<std::vector<float>> vvf = parseVVF(footprint_string, error);

    if (error != "") {
        LOG(ERROR) << "Error parsing footprint parameter: " << error.c_str();
        LOG(ERROR) << "Footprint string was  " << footprint_string.c_str();

        return false;
    }

    // convert vvf into points.
    if (vvf.size() < 3) {
        LOG(ERROR) << "You must specify at least three points for the robot "
                      "footprint, reverting to previous footprint.";
        return false;
    }

    footprint.reserve(vvf.size());
    for (unsigned int i = 0; i < vvf.size(); i++) {
        if (vvf[i].size() == 2) {
            automsgs::msgs::geometry_msgs::Point point;
            point.set_x(vvf[i][0]);
            point.set_y(vvf[i][1]);
            point.set_z(0);
            footprint.push_back(point);
        } else {
            LOG(ERROR)
                << " Points in the footprint specification must be pairs of "
                   "numbers. Found a point with "
                << static_cast<int>(vvf[i].size()) << " numbers.";
            return false;
        }
    }

    return true;
}

}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy
