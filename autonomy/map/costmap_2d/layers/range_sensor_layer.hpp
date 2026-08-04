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

#pragma once

#include <chrono>
#include <list>
#include <mutex>
#include <string>
#include <vector>

#include <automsgs/msgs/builtin_interfaces/time.pb.h>
#include <automsgs/msgs/builtin_interfaces/duration.pb.h>
#include <automsgs/msgs/time_utils.hpp>
#include <automsgs/msgs/geometry_msgs/point.pb.h>
#include <automsgs/msgs/geometry_msgs/quaternion.pb.h>
#include <automsgs/msgs/geometry_msgs/pose.pb.h>
#include <automsgs/msgs/geometry_msgs/pose_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/transform.pb.h>
#include <automsgs/msgs/geometry_msgs/transform_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/twist.pb.h>
#include <automsgs/msgs/geometry_msgs/twist_stamped.pb.h>
#include <automsgs/msgs/geometry_msgs/vector3.pb.h>
#include <automsgs/msgs/map_msgs/occupancy_grid.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>
#include <automsgs/msgs/sensor_msgs/laser_scan.pb.h>
#include <automsgs/msgs/sensor_msgs/imu.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud.pb.h>
#include <automsgs/msgs/sensor_msgs/range.pb.h>
#include "autonomy/map/costmap_2d/costmap_layer.hpp"
#include "autonomy/map/costmap_2d/layered_costmap.hpp"
#include "autonomy/transform/buffer.hpp"
#include "autonomy/transform/tf2/buffer_core.h"

namespace autonomy {
namespace map {
namespace costmap_2d {

/**
 * @class RangeSensorLayer
 * @brief Takes in IR/Sonar/similar point measurement sensors and populates in
 * costmap
 */
class RangeSensorLayer : public CostmapLayer
{
public:
    enum class InputSensorType { VARIABLE, FIXED, ALL };

    /**
     * @brief A constructor
     */
    RangeSensorLayer();

    /**
     * @brief Initialization process of layer on startup
     */
    virtual void onInitialize();

    /**
     * @brief Update the bounds of the master costmap by this layer's update
     * dimensions
     * @param robot_x X pose of robot
     * @param robot_y Y pose of robot
     * @param robot_yaw Robot orientation
     * @param min_x X min map coord of the window to update
     * @param min_y Y min map coord of the window to update
     * @param max_x X max map coord of the window to update
     * @param max_y Y max map coord of the window to update
     */
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                              double* min_x, double* min_y, double* max_x,
                              double* max_y);

    /**
     * @brief Update the costs in the master costmap in the window
     * @param master_grid The master costmap grid to update
     * @param min_x X min map coord of the window to update
     * @param min_y Y min map coord of the window to update
     * @param max_x X max map coord of the window to update
     * @param max_y Y max map coord of the window to update
     */
    virtual void updateCosts(Costmap2D& master_grid, int min_i, int min_j,
                             int max_i, int max_j);

    /**
     * @brief Reset this costmap
     */
    virtual void reset();

    /**
     * @brief Deactivate the layer
     */
    virtual void deactivate();

    /**
     * @brief Activate the layer
     */
    virtual void activate();

    /**
     * @brief If clearing operations should be processed on this layer or not
     */
    virtual bool isClearable() {
        return true;
    }

    /**
     * @brief Handle an incoming Range message to populate into costmap
     */
    void bufferIncomingRangeMsg(
        const std::shared_ptr<automsgs::msgs::sensor_msgs::Range> range_message);

    /** @brief Inject range measurement from an external bridge (e.g. ROS). */
    void feedRange(const automsgs::msgs::sensor_msgs::Range& range);

protected:
    /**
     * @brief Processes all sensors into the costmap buffered from callbacks
     */
    void updateCostmap();
    /**
     * @brief Update the actual costmap with the values processed
     */
    void updateCostmap(automsgs::msgs::sensor_msgs::Range& range_message,
                       bool clear_sensor_cone);

    /**
     * @brief Process general incoming range sensor data. If min=max ranges,
     * fixed processor callback is used, else uses variable callback
     */
    void processRangeMsg(automsgs::msgs::sensor_msgs::Range& range_message);

    /**
     * @brief Process fixed range incoming range sensor data
     */
    void processFixedRangeMsg(automsgs::msgs::sensor_msgs::Range& range_message);

    /**
     * @brief Process variable range incoming range sensor data
     */
    void processVariableRangeMsg(automsgs::msgs::sensor_msgs::Range& range_message);

    /**
     * @brief Reset the angle min/max x, and min/max y values
     */
    void resetRange();

    /**
     * @brief Get the gamma value for an angle, theta
     */
    inline double gamma(double theta);

    /**
     * @brief Get the delta value for an angle, phi
     */
    inline double delta(double phi);

    /**
     * @brief Apply the sensor model of the layer for range sensors
     */
    inline double sensor_model(double r, double phi, double theta);

    /**
     * @brief Get angles
     */
    inline void get_deltas(double angle, double* dx, double* dy);

    /**
     * @brief Update the cost in a cell with information
     */
    inline void update_cell(double ox, double oy, double ot, double r,
                            double nx, double ny, bool clear);

    /**
     * @brief Find probability value of a cost
     */
    inline double to_prob(unsigned char c) {
        return static_cast<double>(c) / LETHAL_OBSTACLE;
    }

    /**
     * @brief Find cost value of a probability
     */
    inline unsigned char to_cost(double p) {
        return static_cast<unsigned char>(p * LETHAL_OBSTACLE);
    }

    std::function<void(automsgs::msgs::sensor_msgs::Range& range_message)>
        processRangeMessageFunc_;
    std::mutex range_message_mutex_;
    std::list<automsgs::msgs::sensor_msgs::Range> range_msgs_buffer_;

    double max_angle_, phi_v_;
    double inflate_cone_;
    std::string global_frame_;

    double clear_threshold_, mark_threshold_;
    bool clear_on_max_reading_;
    bool was_reset_;

    transform::tf2::Duration transform_tolerance_;
    double no_readings_timeout_{0.0};
    std::chrono::steady_clock::time_point last_reading_steady_;
    unsigned int buffered_readings_{0};
    // std::vector<rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr>
    // range_subs_;
    double min_x_, min_y_, max_x_, max_y_;

    /**
     * @brief Find the area of 3 points of a triangle
     */
    float area(int x1, int y1, int x2, int y2, int x3, int y3) {
        return fabs((x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)) / 2.0);
    }

    /**
     * @brief Find the cross product of 3 vectors, A,B,C
     */
    int orient2d(int Ax, int Ay, int Bx, int By, int Cx, int Cy) {
        return (Bx - Ax) * (Cy - Ay) - (By - Ay) * (Cx - Ax);
    }
};

}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy