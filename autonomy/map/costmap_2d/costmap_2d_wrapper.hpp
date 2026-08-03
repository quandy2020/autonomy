#include <automsgs/msgs/sensor_msgs/range.pb.h>
/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *********************************************************************/

#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "autonomy/common/lua_parameter_dictionary.hpp"
#include "autonomy/common/macros.hpp"
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
#include <automsgs/msgs/map_msgs/map_msgs.pb.h>
#include <automsgs/msgs/nav_msgs/occupancy_grid.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud2.pb.h>
#include <automsgs/msgs/sensor_msgs/laser_scan.pb.h>
#include <automsgs/msgs/sensor_msgs/imu.pb.h>
#include <automsgs/msgs/sensor_msgs/point_cloud.pb.h>
#include "autonomy/map/common/map_interface.hpp"
#include "autonomy/map/costmap_2d/costmap_2d.hpp"
#include "autonomy/map/costmap_2d/footprint.hpp"
#include "autonomy/map/costmap_2d/layered_costmap.hpp"
#include "autonomy/map/costmap_2d/map_io.hpp"
#include "autonomy/map/proto/map_options.pb.h"
#include "autonomy/transform/buffer.hpp"
#include "autonomy/transform/tf2/utils.h"

namespace autonomy {
namespace map {
namespace costmap_2d {

class Costmap2DWrapper : public common::MapInterface
{
public:
    using TfBuffer = autonomy::transform::Buffer;

    /**
     * Define Costmap2DWrapper::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(Costmap2DWrapper)

    /**
     * @brief A constructor for nautonomy::map::costmap_2d::Costmap2DWrapper
     * @param options Additional options to control creation of the node.
     * @param name The name of the costmap wrapper
     */
    Costmap2DWrapper(const proto::Costmap2DOptions& options,
                     const std::string& name = "");

    /**
     * @brief A Destructor for autonomy::map::costmap_2d::Costmap2DWrapper
     */
    ~Costmap2DWrapper();

    /**
     * @brief Common initialization for constructors
     */
    void init();

    /**
     * @brief  Subscribes to sensor topics if necessary and starts costmap
     * updates, can be called to restart the costmap after calls to either
     * stop() or pause()
     */
    void Start() override;

    /**
     * @brief  Stops costmap updates and unsubscribes from sensor topics
     */
    void Stop() override;

    /**
     * @brief  Stops the costmap from updating, but sensor data still comes in
     * over the wire
     */
    void Pause() override;

    /**
     * @brief  Resumes costmap updates
     */
    void Resume() override;

    /**
     * @brief Update the map with the layered costmap / plugins
     */
    void updateMap();

    /**
     * @brief Reset each individual layer
     */
    void resetLayers();

    void setRobotFootprint(
        const std::vector<automsgs::msgs::geometry_msgs::Point>& points);

    void setRobotFootprintPolygon(
        const std::shared_ptr<automsgs::msgs::geometry_msgs::Polygon> footprint);

    void getOrientedFootprint(
        std::vector<automsgs::msgs::geometry_msgs::Point>& oriented_footprint);

    /**
     * @brief Same as getLayeredCostmap()->isCurrent().
     */
    bool isCurrent() {
        return layered_costmap_->isCurrent();
    }

    /**
     * @brief Get the pose of the robot in the global frame of the costmap
     * @param global_pose Will be set to the pose of the robot in the global
     * frame of the costmap
     * @return True if the pose was set successfully, false otherwise
     */
    bool getRobotPose(automsgs::msgs::geometry_msgs::PoseStamped& global_pose);

    /**
     * @brief Transform the input_pose in the global frame of the costmap
     * @param input_pose pose to be transformed
     * @param transformed_pose pose transformed
     * @return True if the pose was transformed successfully, false otherwise
     */
    bool transformPoseToGlobalFrame(
        const automsgs::msgs::geometry_msgs::PoseStamped& input_pose,
        automsgs::msgs::geometry_msgs::PoseStamped& transformed_pose);

    /**
     * @brief Returns costmap name
     */
    std::string getName() const {
        return name_;
    }

    /**
     * @brief Returns the delay in transform (tf) data that is tolerable in
     * seconds
     */
    double getTransformTolerance() const {
        return transform_tolerance_;
    }

    /**
     * @brief Return a pointer to the "master" costmap which receives updates
     * from all the layers.
     *
     * Same as calling getLayeredCostmap()->getCostmap().
     */
    Costmap2D* getCostmap() {
        return layered_costmap_->getCostmap();
    }

    /**
     * @brief Check if the costmap has been updated at least once and is ready
     * for use.
     * @return True if the costmap is ready, false otherwise.
     */
    bool isReady() const {
        return ready_;
    }

    /**
     * @brief  Returns the global frame of the costmap
     * @return The global frame of the costmap
     */
    std::string getGlobalFrameID() {
        return global_frame_;
    }

    /** Align costmap TF frame with runtime overrides (e.g. ROS global_frame:=odom). */
    void setGlobalFrameID(const std::string& global_frame);

    /** Align robot base frame with runtime (e.g. base_footprint in simulation). */
    void setRobotBaseFrameID(const std::string& robot_base_frame);

    /**
     * @brief  Returns the local frame of the costmap
     * @return The local frame of the costmap
     */
    std::string getBaseFrameID() {
        return robot_base_frame_;
    }

    /**
     * @brief Get the layered costmap object used in the node
     */
    LayeredCostmap* getLayeredCostmap() {
        return layered_costmap_.get();
    }

    /**
     * @brief Returns the current padded footprint as a
     * geometry_msgs::msg::Polygon.
     */
    automsgs::msgs::geometry_msgs::Polygon getRobotFootprintPolygon() {
        return toPolygon(padded_footprint_);
    }

    /**
     * @brief Return the current footprint of the robot as a vector of points.
     *
     * This version of the footprint is padded by the footprint_padding_
     * distance, set in the rosparam "footprint_padding".
     *
     * The footprint initially comes from the rosparam "footprint" but
     * can be overwritten by dynamic reconfigure or by messages received
     * on the "footprint" topic. */
    std::vector<automsgs::msgs::geometry_msgs::Point> getRobotFootprint() {
        return padded_footprint_;
    }

    /** @brief Return the current unpadded footprint of the robot as a vector of
     * points.
     *
     * This is the raw version of the footprint without padding.
     *
     * The footprint initially comes from the rosparam "footprint" but
     * can be overwritten by dynamic reconfigure or by messages received
     * on the "footprint" topic. */
    std::vector<automsgs::msgs::geometry_msgs::Point> getUnpaddedRobotFootprint() {
        return unpadded_footprint_;
    }

    /**
     * @brief Load the map metadata into the costmap
     * @param map_meta_data The map metadata to load
     * @return Whether the map was loaded successfully
     */
    bool loadMap(const std::string& filename);

    /**
     * @brief Apply an OccupancyGrid from MapServer or SLAM into StaticLayer.
     */
    bool applyOccupancyGrid(const automsgs::msgs::map_msgs::OccupancyGrid& grid);

    /**
     * @brief Publish the map to the topic
     */
    void publishMap();

    /**
     * @brief Inject laser scan into ObstacleLayer plugins (ROS bridge entry).
     */
    void feedLaserScan(const automsgs::msgs::sensor_msgs::LaserScan& scan);

    void feedPointCloud2(const automsgs::msgs::sensor_msgs::PointCloud2& cloud);

    void feedRange(const automsgs::msgs::sensor_msgs::Range& range);

    /**
     * @brief Fill @p grid with the latest costmap as an OccupancyGrid snapshot.
     * @return false if the costmap is not initialized.
     */
    bool snapshotOccupancyGrid(automsgs::msgs::map_msgs::OccupancyGrid& grid);

    /**
     * @brief  Get the costmap's use_radius_ parameter, corresponding to
     * whether the footprint for the robot is a circle with radius robot_radius_
     * or an arbitrarily defined footprint in footprint_.
     * @return  use_radius_
     */
    bool getUseRadius() {
        return use_radius_;
    }

    /**
     * @brief  Get the costmap's robot_radius_ parameter, corresponding to
     * raidus of the robot footprint when it is defined as as circle
     * (i.e. when use_radius_ == true).
     * @return  robot_radius_
     */
    double getRobotRadius() {
        return robot_radius_;
    }

protected:
    /** @brief Instantiate and register configured layer plugins. */
    void loadPlugins();

    /** @brief Push occupancy_grid_ into StaticLayer or the master costmap. */
    void applyLoadedOccupancyGrid(const automsgs::msgs::map_msgs::OccupancyGrid& grid);

    /** @brief Activate or deactivate all loaded layer plugins. */
    void setPluginsActive(bool active);

    std::unique_ptr<LayeredCostmap> layered_costmap_{nullptr};
    std::string name_;

    /**
     * @brief Function on timer for costmap update
     */
    void mapUpdateLoop(double frequency);

    bool map_update_thread_shutdown_{false};
    std::atomic<bool> stop_updates_{false};
    std::atomic<bool> initialized_{false};
    std::atomic<bool> stopped_{true};
    std::mutex _dynamic_parameter_mutex;
    std::unique_ptr<std::thread>
        map_update_thread_;  ///< @brief A thread for updating the map
    automsgs::msgs::builtin_interfaces::Time last_publish_ = automsgs::msgs::builtin_interfaces::ZeroTime();
    // builtin_interfaces::builtin_interfaces::Duration publish_cycle_{1, 0};

    bool always_send_full_costmap_{false};
    std::string footprint_;
    float footprint_padding_{0};
    std::string global_frame_;  ///< The global frame for the costmap
    int map_height_meters_{0};
    double map_publish_frequency_{0};
    double map_update_frequency_{0};
    int map_width_meters_{0};
    double origin_x_{0};
    double origin_y_{0};
    std::vector<std::string> default_plugins_;
    std::vector<std::string> default_types_;
    std::vector<std::string> plugin_names_;
    std::vector<std::string> plugin_types_;
    std::vector<std::string> filter_names_;
    std::vector<std::string> filter_types_;
    double resolution_{0};
    std::string robot_base_frame_;  ///< The frame_id of the robot base
    double robot_radius_;
    bool rolling_window_{
        false};  ///< Whether to use a rolling window version of the costmap
    bool track_unknown_space_{false};
    double transform_tolerance_{0};  ///< The timeout before transform errors
    double initial_transform_timeout_{
        0};  ///< The timeout before activation of the node errors
    double map_vis_z_{
        0};  ///< The height of map, allows to avoid flickering at -0.008

    bool is_lifecycle_follower_{
        true};  ///< whether is a child-LifecycleNode or an independent node
    bool ready_{false};  ///< whether the costmap has been updated at least once

    // Map data
    automsgs::msgs::map_msgs::OccupancyGrid occupancy_grid_;

    // Derived parameters
    bool use_radius_{false};
    bool map_loaded_{false};  // 标记地图是否已加载，避免重复加载
    std::vector<automsgs::msgs::geometry_msgs::Point> unpadded_footprint_;
    std::vector<automsgs::msgs::geometry_msgs::Point> padded_footprint_;

    // options for costmap 2D
    proto::Costmap2DOptions options_;
};

}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy
