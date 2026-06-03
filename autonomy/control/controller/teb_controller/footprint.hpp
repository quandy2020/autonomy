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

#include <memory>
#include <vector>

#include "autonomy/control/controller/teb_controller/geometry/obstacle.hpp"
#include "autonomy/control/controller/teb_controller/pose2d_utils.hpp"
#include "autonomy/macros.hpp"

namespace autonomy {
namespace control {
namespace teb_controller {

/**
 * @brief Robot footprint interface for obstacle distance queries
 *
 * Provides geometric models of the robot body used to compute clearance to
 * static and dynamic obstacles during TEB optimization.
 */
class RobotFootprint
{
public:
    /**
     * Define RobotFootprint::SharedPtr type
     */
    AUTONOMY_SHARED_PTR_DEFINITIONS(RobotFootprint);

    /**
     * @brief Destructor for RobotFootprint
     */
    virtual ~RobotFootprint() = default;

    /**
     * @brief Computes the minimum distance from the robot to an obstacle
     *
     * @param current_pose Current robot pose in the planning frame
     * @param obstacle Obstacle to measure against
     * @return Signed or unsigned clearance depending on the footprint model
     */
    virtual double CalculateDistance(const Pose2D& current_pose,
                                     const Obstacle* obstacle) const = 0;

    /**
     * @brief Estimates spatio-temporal distance assuming constant obstacle motion
     *
     * Predicts obstacle motion over time t and returns the minimum distance
     * between the transformed robot footprint and the obstacle.
     *
     * @param current_pose Current robot pose in the planning frame
     * @param obstacle Obstacle to measure against
     * @param t Prediction horizon [s]
     * @return Estimated clearance at time t
     */
    virtual double EstimateSpatioTemporalDistance(const Pose2D& current_pose,
                                                  const Obstacle* obstacle,
                                                  double t) const = 0;

    /**
     * @brief Appends visualization markers for the robot footprint
     *
     * Default implementation does nothing. Derived classes may populate
     * markers for debug visualization.
     *
     * @param current_pose Current robot pose in the planning frame
     * @param markers Marker list to append to
     * @param color Color applied to generated markers
     */
    virtual void VisualizeRobot(
        const Pose2D& current_pose,
        std::vector<autonomy::commsgs::visualization_msgs::Marker>& markers,
        const autonomy::commsgs::std_msgs::ColorRGBA& color) const {}

    /**
     * @brief Returns the inscribed radius of the footprint
     *
     * @return Inscribed radius [m]
     */
    virtual double GetInscribedRadius() = 0;
};

/**
 * @brief Point robot footprint
 *
 * Models the robot as a point at the pose origin with zero inscribed radius.
 */
class PointRobotFootprint : public RobotFootprint
{
public:
    /**
     * Define PointRobotFootprint::SharedPtr type
     */
    AUTONOMY_SHARED_PTR_DEFINITIONS(PointRobotFootprint);

    /**
     * @brief Computes the minimum distance from the robot point to an obstacle
     *
     * @param current_pose Current robot pose in the planning frame
     * @param obstacle Obstacle to measure against
     * @return Minimum distance from the pose position to the obstacle
     */
    double CalculateDistance(const Pose2D& current_pose,
                             const Obstacle* obstacle) const override;

    /**
     * @brief Estimates spatio-temporal distance for a point robot
     *
     * @param current_pose Current robot pose in the planning frame
     * @param obstacle Obstacle to measure against
     * @param t Prediction horizon [s]
     * @return Estimated clearance at time t
     */
    double EstimateSpatioTemporalDistance(const Pose2D& current_pose,
                                          const Obstacle* obstacle,
                                          double t) const override;

    /**
     * @brief Returns the inscribed radius of the point footprint
     *
     * @return Always zero
     */
    double GetInscribedRadius() override;
};

/**
 * @brief Circular robot footprint
 *
 * Models the robot as a disc centered at the pose origin.
 */
class CircularRobotFootprint : public RobotFootprint
{
public:
    /**
     * Define CircularRobotFootprint::SharedPtr type
     */
    AUTONOMY_SHARED_PTR_DEFINITIONS(CircularRobotFootprint);

    /**
     * @brief Constructor for CircularRobotFootprint
     *
     * @param radius Inscribed radius [m]
     */
    explicit CircularRobotFootprint(double radius);

    /**
     * @brief Updates the footprint radius
     *
     * @param radius Inscribed radius [m]
     */
    void SetRadius(double radius);

    /**
     * @brief Computes clearance between the robot disc and an obstacle
     *
     * @param current_pose Current robot pose in the planning frame
     * @param obstacle Obstacle to measure against
     * @return Distance to obstacle minus the inscribed radius
     */
    double CalculateDistance(const Pose2D& current_pose,
                             const Obstacle* obstacle) const override;

    /**
     * @brief Estimates spatio-temporal clearance for a circular footprint
     *
     * @param current_pose Current robot pose in the planning frame
     * @param obstacle Obstacle to measure against
     * @param t Prediction horizon [s]
     * @return Estimated clearance at time t minus the inscribed radius
     */
    double EstimateSpatioTemporalDistance(const Pose2D& current_pose,
                                          const Obstacle* obstacle,
                                          double t) const override;

    /**
     * @brief Appends a cylinder marker representing the robot disc
     *
     * @param current_pose Current robot pose in the planning frame
     * @param markers Marker list to append to
     * @param color Color applied to the marker
     */
    void VisualizeRobot(
        const Pose2D& current_pose,
        std::vector<autonomy::commsgs::visualization_msgs::Marker>& markers,
        const autonomy::commsgs::std_msgs::ColorRGBA& color) const override;

    /**
     * @brief Returns the inscribed radius of the circular footprint
     *
     * @return Inscribed radius [m]
     */
    double GetInscribedRadius() override;

private:
    double radius_;
};

/**
 * @brief Two-circle robot footprint
 *
 * Approximates the robot body with front and rear circles offset along the
 * robot heading.
 */
class TwoCirclesRobotFootprint : public RobotFootprint
{
public:
    /**
     * Define TwoCirclesRobotFootprint::SharedPtr type
     */
    AUTONOMY_SHARED_PTR_DEFINITIONS(TwoCirclesRobotFootprint);

    /**
     * @brief Constructor for TwoCirclesRobotFootprint
     *
     * @param front_offset Longitudinal offset of the front circle center [m]
     * @param front_radius Radius of the front circle [m]
     * @param rear_offset Longitudinal offset of the rear circle center [m]
     * @param rear_radius Radius of the rear circle [m]
     */
    TwoCirclesRobotFootprint(double front_offset, double front_radius,
                             double rear_offset, double rear_radius);

    /**
     * @brief Destructor for TwoCirclesRobotFootprint
     */
    ~TwoCirclesRobotFootprint() override = default;

    /**
     * @brief Updates front and rear circle parameters
     *
     * @param front_offset Longitudinal offset of the front circle center [m]
     * @param front_radius Radius of the front circle [m]
     * @param rear_offset Longitudinal offset of the rear circle center [m]
     * @param rear_radius Radius of the rear circle [m]
     */
    void SetParameters(double front_offset, double front_radius,
                       double rear_offset, double rear_radius) override {
        front_offset_ = front_offset;
        front_radius_ = front_radius;
        rear_offset_ = rear_offset;
        rear_radius_ = rear_radius;
    }

    /**
     * @brief Computes clearance using the minimum distance of both circles
     *
     * @param current_pose Current robot pose in the planning frame
     * @param obstacle Obstacle to measure against
     * @return Minimum clearance over the front and rear circles
     */
    double CalculateDistance(const Pose2D& current_pose,
                             const Obstacle* obstacle) const override;

    /**
     * @brief Estimates spatio-temporal clearance for the two-circle model
     *
     * @param current_pose Current robot pose in the planning frame
     * @param obstacle Obstacle to measure against
     * @param t Prediction horizon [s]
     * @return Minimum estimated clearance over the front and rear circles
     */
    double EstimateSpatioTemporalDistance(const Pose2D& current_pose,
                                          const Obstacle* obstacle,
                                          double t) const override;

    /**
     * @brief Appends cylinder markers for the front and rear circles
     *
     * @param current_pose Current robot pose in the planning frame
     * @param markers Marker list to append to
     * @param color Color applied to the markers
     */
    void VisualizeRobot(
        const Pose2D& current_pose,
        std::vector<autonomy::commsgs::visualization_msgs::Marker>& markers,
        const autonomy::commsgs::std_msgs::ColorRGBA& color) const override;

    /**
     * @brief Returns a conservative inscribed radius of the footprint
     *
     * @return Approximate inscribed radius [m]
     */
    double GetInscribedRadius() override;

private:
    double front_offset_;
    double front_radius_;
    double rear_offset_;
    double rear_radius_;
};

/**
 * @brief Line segment robot footprint
 *
 * Models the robot as a line segment defined in the robot frame and
 * transformed into the planning frame by the current pose.
 */
class LineRobotFootprint : public RobotFootprint
{
public:
    /**
     * Define LineRobotFootprint::SharedPtr type
     */
    AUTONOMY_SHARED_PTR_DEFINITIONS(LineRobotFootprint);

    /**
     * @brief Constructor for LineRobotFootprint
     *
     * Line endpoints are expressed in the robot frame with origin at the pose
     * center.
     *
     * @param line_start Start point of the segment in the robot frame
     * @param line_end End point of the segment in the robot frame
     */
    LineRobotFootprint(
        const autonomy::commsgs::geometry_msgs::Point& line_start,
        const autonomy::commsgs::geometry_msgs::Point& line_end);

    /**
     * @brief Constructor for LineRobotFootprint
     *
     * @param line_start Start point of the segment in the robot frame
     * @param line_end End point of the segment in the robot frame
     */
    LineRobotFootprint(const Point& line_start, const Point& line_end);

    /**
     * @brief Sets the line segment in the robot frame
     *
     * @param line_start Start point of the segment in the robot frame
     * @param line_end End point of the segment in the robot frame
     */
    void SetLine(const autonomy::commsgs::geometry_msgs::Point& line_start,
                 const autonomy::commsgs::geometry_msgs::Point& line_end);

    /**
     * @brief Sets the line segment in the robot frame
     *
     * @param line_start Start point of the segment in the robot frame
     * @param line_end End point of the segment in the robot frame
     */
    void SetLine(const Point& line_start, const Point& line_end);

    /**
     * @brief Computes clearance between the transformed line and an obstacle
     *
     * @param current_pose Current robot pose in the planning frame
     * @param obstacle Obstacle to measure against
     * @return Minimum distance from the line segment to the obstacle
     */
    double CalculateDistance(const Pose2D& current_pose,
                             const Obstacle* obstacle) const override;

    /**
     * @brief Estimates spatio-temporal clearance for a line footprint
     *
     * @param current_pose Current robot pose in the planning frame
     * @param obstacle Obstacle to measure against
     * @param t Prediction horizon [s]
     * @return Estimated clearance at time t
     */
    double EstimateSpatioTemporalDistance(const Pose2D& current_pose,
                                          const Obstacle* obstacle,
                                          double t) const override;

    /**
     * @brief Appends a line-strip marker for the robot segment
     *
     * @param current_pose Current robot pose in the planning frame
     * @param markers Marker list to append to
     * @param color Color applied to the marker
     */
    void VisualizeRobot(
        const Pose2D& current_pose,
        std::vector<autonomy::commsgs::visualization_msgs::Marker>& markers,
        const autonomy::commsgs::std_msgs::ColorRGBA& color) const override;

    /**
     * @brief Returns the inscribed radius of the line footprint
     *
     * @return Always zero
     */
    double GetInscribedRadius() override;

private:
    /**
     * @brief Transforms the robot-frame line segment into the planning frame
     *
     * @param current_pose Current robot pose in the planning frame
     * @param line_start_world Output start point in the planning frame
     * @param line_end_world Output end point in the planning frame
     */
    void TransformToWorld(const Pose2D& current_pose, Point& line_start_world,
                          Point& line_end_world) const;

    Point line_start_{};
    Point line_end_{};
};

/**
 * @brief Polygon robot footprint
 *
 * Models the robot as a closed polygon defined in the robot frame and
 * transformed into the planning frame by the current pose.
 */
class PolygonRobotFootprint : public RobotFootprint
{
public:
    /**
     * Define PolygonRobotFootprint::SharedPtr type
     */
    AUTONOMY_SHARED_PTR_DEFINITIONS(PolygonRobotFootprint);

    /**
     * @brief Constructor for PolygonRobotFootprint
     *
     * Vertices are expressed in the robot frame with origin at the pose center.
     * Do not repeat the first vertex at the end of the list.
     *
     * @param vertices Polygon vertices in the robot frame
     */
    explicit PolygonRobotFootprint(const Point2dContainer& vertices);

    /**
     * @brief Destructor for PolygonRobotFootprint
     */
    ~PolygonRobotFootprint() override = default;

    /**
     * @brief Sets polygon vertices in the robot frame
     *
     * @param vertices Polygon vertices in the robot frame
     */
    void SetVertices(const Point2dContainer& vertices);

    /**
     * @brief Computes clearance between the transformed polygon and an obstacle
     *
     * @param current_pose Current robot pose in the planning frame
     * @param obstacle Obstacle to measure against
     * @return Minimum distance from the polygon to the obstacle
     */
    double CalculateDistance(const Pose2D& current_pose,
                             const Obstacle* obstacle) const override;

    /**
     * @brief Estimates spatio-temporal clearance for a polygon footprint
     *
     * @param current_pose Current robot pose in the planning frame
     * @param obstacle Obstacle to measure against
     * @param t Prediction horizon [s]
     * @return Estimated clearance at time t
     */
    double EstimateSpatioTemporalDistance(const Pose2D& current_pose,
                                          const Obstacle* obstacle,
                                          double t) const override;

    /**
     * @brief Appends a line-strip marker for the robot polygon
     *
     * @param current_pose Current robot pose in the planning frame
     * @param markers Marker list to append to
     * @param color Color applied to the marker
     */
    void VisualizeRobot(
        const Pose2D& current_pose,
        std::vector<autonomy::commsgs::visualization_msgs::Marker>& markers,
        const autonomy::commsgs::std_msgs::ColorRGBA& color) const override;

    /**
     * @brief Returns the inscribed radius of the polygon footprint
     *
     * Computes the minimum distance from the robot origin to polygon edges and
     * vertices.
     *
     * @return Inscribed radius [m]
     */
    double GetInscribedRadius() override;

private:
    /**
     * @brief Transforms robot-frame polygon vertices into the planning frame
     *
     * @param current_pose Current robot pose in the planning frame
     * @param polygon_world Output vertices in the planning frame
     */
    void TransformToWorld(const Pose2D& current_pose,
                          Point2dContainer& polygon_world) const;

    Point2dContainer vertices_;
};

using RobotFootprintPtr = RobotFootprint::SharedPtr;

}  // namespace teb_controller
}  // namespace control
}  // namespace autonomy
