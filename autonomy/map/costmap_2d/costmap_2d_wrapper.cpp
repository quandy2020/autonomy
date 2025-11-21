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

#include "autonomy/map/costmap_2d/costmap_2d_wrapper.hpp"
#include "autonomy/common/logging.hpp"
namespace autonomy {
namespace map {
namespace costmap_2d {

Costmap2DWrapper::Costmap2DWrapper(
    const proto::Costmap2DOptions& options, 
    const std::string& name)
    : options_{options},
      default_plugins_ {"static_layer", "obstacle_layer", "inflation_layer"},
      default_types_ {
        "costmap_2d::StaticLayer",
        "costmap_2d::ObstacleLayer",
        "costmap_2d::InflationLayer"
      }
{
    // declare_parameter("map_topic", rclcpp::ParameterValue(std::string("map")));
    is_lifecycle_follower_ = false;
    init();

}

Costmap2DWrapper::~Costmap2DWrapper()
{

}

void Costmap2DWrapper::init()
{
    LOG(INFO) << "Creating Costmap";

    // declare_parameter("always_send_full_costmap", rclcpp::ParameterValue(false));
    // declare_parameter("map_vis_z", rclcpp::ParameterValue(0.0));
    // declare_parameter("footprint_padding", rclcpp::ParameterValue(0.01f));
    // declare_parameter("footprint", rclcpp::ParameterValue(std::string("[]")));
    // declare_parameter("global_frame", rclcpp::ParameterValue(std::string("map")));
    // declare_parameter("height", rclcpp::ParameterValue(5));
    // declare_parameter("width", rclcpp::ParameterValue(5));
    // declare_parameter("lethal_cost_threshold", rclcpp::ParameterValue(100));
    // declare_parameter("observation_sources", rclcpp::ParameterValue(std::string("")));
    // declare_parameter("origin_x", rclcpp::ParameterValue(0.0));
    // declare_parameter("origin_y", rclcpp::ParameterValue(0.0));
    // declare_parameter("plugins", rclcpp::ParameterValue(default_plugins_));
    // declare_parameter("filters", rclcpp::ParameterValue(std::vector<std::string>()));
    // declare_parameter("publish_frequency", rclcpp::ParameterValue(1.0));
    // declare_parameter("resolution", rclcpp::ParameterValue(0.1));
    // declare_parameter("robot_base_frame", rclcpp::ParameterValue(std::string("base_link")));
    // declare_parameter("robot_radius", rclcpp::ParameterValue(0.1));
    // declare_parameter("rolling_window", rclcpp::ParameterValue(false));
    // declare_parameter("track_unknown_space", rclcpp::ParameterValue(false));
    // declare_parameter("transform_tolerance", rclcpp::ParameterValue(0.3));
    // declare_parameter("initial_transform_timeout", rclcpp::ParameterValue(60.0));
    // declare_parameter("trinary_costmap", rclcpp::ParameterValue(true));
    // declare_parameter("unknown_cost_value", rclcpp::ParameterValue(static_cast<unsigned char>(0xff)));
    // declare_parameter("update_frequency", rclcpp::ParameterValue(5.0));
    // declare_parameter("use_maximum", rclcpp::ParameterValue(false));

    // Create the costmap itself
    layered_costmap_ = std::make_unique<LayeredCostmap>(
        global_frame_, rolling_window_, track_unknown_space_);

    if (!layered_costmap_->isSizeLocked()) {
        layered_costmap_->resizeMap(
            (unsigned int)(map_width_meters_ / resolution_),
            (unsigned int)(map_height_meters_ / resolution_), resolution_, origin_x_, origin_y_);
    }

    // Then load and add the plug-ins to the costmap
    for (unsigned int i = 0; i < plugin_names_.size(); ++i) {
        LOG(INFO) << "Using plugin " << plugin_names_[i];

        // std::shared_ptr<Layer> plugin = plugin_loader_.createSharedInstance(plugin_types_[i]);

        // // lock the costmap because no update is allowed until the plugin is initialized
        // std::unique_lock<Costmap2D::mutex_t> lock(*(layered_costmap_->getCostmap()->getMutex()));
        // layered_costmap_->addPlugin(plugin);
        // // TODO(mjeronimo): instead of get(), use a shared ptr
        // plugin->initialize(layered_costmap_.get(), plugin_names_[i], tf_buffer_.get(),
        //     shared_from_this(), callback_group_);
        // lock.unlock();
        LOG(INFO) << "Initialized plugin " << plugin_names_[i];
    }

    // and costmap filters as well
    for (unsigned int i = 0; i < filter_names_.size(); ++i) {
        LOG(INFO) << "Using costmap filter " << filter_names_[i];

        // std::shared_ptr<Layer> filter = plugin_loader_.createSharedInstance(filter_types_[i]);

        // // lock the costmap because no update is allowed until the filter is initialized
        // std::unique_lock<Costmap2D::mutex_t> lock(*(layered_costmap_->getCostmap()->getMutex()));

        // layered_costmap_->addFilter(filter);

        // filter->initialize(layered_costmap_.get(), filter_names_[i], tf_buffer_.get(),
        //     shared_from_this(), callback_group_);

        // lock.unlock();

        LOG(INFO) << "Initialized costmapfilter " << filter_names_[i];
    }

    auto layers = layered_costmap_->getPlugins();

    // for (auto & layer : *layers) {
    //     auto costmap_layer = std::dynamic_pointer_cast<CostmapLayer>(layer);
    //     if (costmap_layer != nullptr) {
    //         layer_publishers_.emplace_back(
    //             std::make_unique<Costmap2DPublisher>(
    //             shared_from_this(),
    //             costmap_layer.get(), global_frame_,
    //             layer->getName(), always_send_full_costmap_, map_vis_z_)
    //         );
    //     }
    // }

    // Set the footprint
    if (use_radius_) {
        setRobotFootprint(makeFootprintFromRadius(robot_radius_));
    } else {
        std::vector<commsgs::geometry_msgs::Point> new_footprint;
        makeFootprintFromString(footprint_, new_footprint);
        setRobotFootprint(new_footprint);
    }
}

void Costmap2DWrapper::Start()
{
    std::string filename = "/workspace/autonomy/configuration_files/map/turtlebot3_house.yaml";
    loadMap(filename);

    while (true) {
        publishMap();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    // LOG(INFO) << "start";
    // std::vector<std::shared_ptr<Layer>>* plugins = layered_costmap_->getPlugins();
    // std::vector<std::shared_ptr<Layer>>* filters = layered_costmap_->getFilters();

    // // check if we're stopped or just paused
    // if (stopped_) {
    //     // if we're stopped we need to re-subscribe to topics
    //     for (auto plugin = plugins->begin(); plugin != plugins->end(); ++plugin) {
    //         // (*plugin)->activate();
    //     }
    //     for (auto filter = filters->begin(); filter != filters->end(); ++filter) {
    //         // (*filter)->activate();
    //     }
    //     stopped_ = false;
    // }
    // stop_updates_ = false;

    // // // block until the costmap is re-initialized.. meaning one update cycle has run
    // // rclcpp::Rate r(20.0);
    // // while (rclcpp::ok() && !initialized_) {
    // //     RCLCPP_DEBUG(get_logger(), "Sleeping, waiting for initialized_");
    // //     r.sleep();
    // // }

    // // Create a thread to handle updating the map
    // stopped_ = true;  // to active plugins
    // stop_updates_ = false;
    // map_update_thread_shutdown_ = false;
    // map_update_thread_ = std::make_unique<std::thread>(
    //     std::bind(&Costmap2DWrapper::mapUpdateLoop, this, map_update_frequency_));
}

void Costmap2DWrapper::Stop()
{
    stop_updates_ = true;

    // layered_costmap_ is set only if on_configure has been called
    if (layered_costmap_) {
        std::vector<std::shared_ptr<Layer>>* plugins = layered_costmap_->getPlugins();
        std::vector<std::shared_ptr<Layer>>* filters = layered_costmap_->getFilters();

        // unsubscribe from topics
        for (auto plugin = plugins->begin(); plugin != plugins->end(); ++plugin) {
            // (*plugin)->deactivate();
        }
        for (auto filter = filters->begin(); filter != filters->end(); ++filter) {
            // (*filter)->deactivate();
        }
    }
    initialized_ = false;
    stopped_ = true;
}

void Costmap2DWrapper::Pause()
{
    stop_updates_ = true;
    initialized_ = false;
}

void Costmap2DWrapper::Resume()
{
    stop_updates_ = false;
    // block until the costmap is re-initialized.. meaning one update cycle has run
    while (!initialized_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void Costmap2DWrapper::resetLayers()
{
    Costmap2D* top = layered_costmap_->getCostmap();
    top->resetMap(0, 0, top->getSizeInCellsX(), top->getSizeInCellsY());

    // Reset each of the plugins
    std::vector<std::shared_ptr<Layer>>* plugins = layered_costmap_->getPlugins();
    std::vector<std::shared_ptr<Layer>>* filters = layered_costmap_->getFilters();
    for (auto plugin = plugins->begin(); plugin != plugins->end(); ++plugin) {
        (*plugin)->reset();
    }
    for (auto filter = filters->begin(); filter != filters->end(); ++filter) {
        (*filter)->reset();
    }
}

void Costmap2DWrapper::setRobotFootprint(const std::vector<commsgs::geometry_msgs::Point>& points)
{
    unpadded_footprint_ = points;
    padded_footprint_ = points;
    padFootprint(padded_footprint_, footprint_padding_);
    layered_costmap_->setFootprint(padded_footprint_);
}

void Costmap2DWrapper::setRobotFootprintPolygon(const commsgs::geometry_msgs::Polygon::SharedPtr footprint)
{
  setRobotFootprint(toPointVector(footprint));
}

void Costmap2DWrapper::getOrientedFootprint(std::vector<commsgs::geometry_msgs::Point>& oriented_footprint)
{
    commsgs::geometry_msgs::PoseStamped global_pose;
    if (!getRobotPose(global_pose)) {
        return;
    }

    // double yaw = transform::tf2::getYaw(global_pose.pose.orientation);
    // transformFootprint(
    //     global_pose.pose.position.x, global_pose.pose.position.y, yaw,
    //     padded_footprint_, oriented_footprint);
}

void Costmap2DWrapper::mapUpdateLoop(double frequency)
{

}

bool Costmap2DWrapper::getRobotPose(commsgs::geometry_msgs::PoseStamped& global_pose)
{
    return true;
    // return nav2_util::getCurrentPose(
    //     global_pose, *tf_buffer_,
    //     global_frame_, robot_base_frame_, transform_tolerance_);
}

bool Costmap2DWrapper::transformPoseToGlobalFrame(
  const commsgs::geometry_msgs::PoseStamped& input_pose,
  commsgs::geometry_msgs::PoseStamped& transformed_pose)
{
    // if (input_pose.header.frame_id == global_frame_) {
    //     transformed_pose = input_pose;
    //     return true;
    // } else {
    //     return utils::transformPoseInTargetFrame(
    //     input_pose, transformed_pose, *tf_buffer_,
    //     global_frame_, transform_tolerance_);
    // }

    return true;
}

bool Costmap2DWrapper::loadMap(const std::string& filename)
{
    if (loadMapFromYaml(filename, occupancy_grid_) != LOAD_MAP_STATUS::LOAD_MAP_SUCCESS) {
        LOG(ERROR) << "Load yaml file error.";
        return false;
    }
    return true;
}

void Costmap2DWrapper::publishMap()
{
    occupancy_grid_.header.frame_id = global_frame_;
    occupancy_grid_.header.stamp = commsgs::builtin_interfaces::Time::Now();
    LOG(INFO) << "Publish map (publisher disabled).";
}

proto::Costmap2DOptions CreateCostmap2DOptions(
    ::autonomy::common::LuaParameterDictionary* const parameter_dictionary)
{
    proto::Costmap2DOptions options;
    // options.set_map_file(parameter_dictionary->GetString("map_file"));
    return options;
}

}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy
