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
#include <vector>

#include "autonomy/transform/tf2/utils.h"
#include "autonomy/map/proto/map_options.pb.h"

#include "autonomy/common/macros.hpp"
#include "autonomy/common/lua_parameter_dictionary.hpp"
#include "autonomy/commsgs/map_msgs.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/map/common/map_interface.hpp"

namespace autonomy {
namespace map {
namespace costmap_2d {

class Costmap2DWrapper : public common::MapInterface
{
public:
    /**
     * Define Costmap2DWrapper::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(Costmap2DWrapper)

    /**
     * @brief A constructor for nautonomy::map::costmap_2d::Costmap2DWrapper
     * @param options Additional options to control creation of the node.
     */
    Costmap2DWrapper(const proto::Costmap2DOptions& options, const std::string& name = "");

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
    void start();

    /**
     * @brief  Stops costmap updates and unsubscribes from sensor topics
     */
    void stop();

    /**
     * @brief  Stops the costmap from updating, but sensor data still comes in over the wire
     */
    void pause();

    /**
     * @brief  Resumes costmap updates
     */
    void resume();

    /**
     * @brief Update the map with the layered costmap / plugins
     */
    void updateMap();

    /**
     * @brief Reset each individual layer
     */
    void resetLayers();

    /** 
     * @brief Same as getLayeredCostmap()->isCurrent(). 
     */
    bool isCurrent()
    {
        return layered_costmap_->isCurrent();
    }

    /**
     * @brief Get the pose of the robot in the global frame of the costmap
     * @param global_pose Will be set to the pose of the robot in the global frame of the costmap
     * @return True if the pose was set successfully, false otherwise
     */
    bool getRobotPose(commsgs::geometry_msgs::PoseStamped& global_pose);

    /**
     * @brief Transform the input_pose in the global frame of the costmap
     * @param input_pose pose to be transformed
     * @param transformed_pose pose transformed
     * @return True if the pose was transformed successfully, false otherwise
     */
    bool transformPoseToGlobalFrame(
        const geometry_msgs::PoseStamped& input_pose,
        commsgs::geometry_msgs::PoseStamped& transformed_pose);

    /** 
     * @brief Returns costmap name 
     */
    std::string getName() const
    {
        return name_;
    }

    /** 
     * @brief Returns the delay in transform (tf) data that is tolerable in seconds 
     */
    double getTransformTolerance() const
    {
        return transform_tolerance_;
    }

    /**
     * @brief Return a pointer to the "master" costmap which receives updates from all the layers.
     *
     * Same as calling getLayeredCostmap()->getCostmap().
     */
    Costmap2D * getCostmap()
    {
        return layered_costmap_->getCostmap();
    }

    /**
     * @brief  Returns the global frame of the costmap
     * @return The global frame of the costmap
     */
    std::string getGlobalFrameID()
    {
        return global_frame_;
    }

    /**
     * @brief  Returns the local frame of the costmap
     * @return The local frame of the costmap
     */
    std::string getBaseFrameID()
    {
        return robot_base_frame_;
    }

    /**
     * @brief Get the layered costmap object used in the node
     */
    LayeredCostmap* getLayeredCostmap()
    {
        return layered_costmap_.get();
    }

    // /** 
    //  * @brief Returns the current padded footprint as a geometry_msgs::msg::Polygon. 
    //  */
    // geometry_msgs::msg::Polygon getRobotFootprintPolygon()
    // {
    //     return nav2_costmap_2d::toPolygon(padded_footprint_);
    // }

    /** 
     * @brief Return the current footprint of the robot as a vector of points.
     *
     * This version of the footprint is padded by the footprint_padding_
     * distance, set in the rosparam "footprint_padding".
     *
     * The footprint initially comes from the rosparam "footprint" but
     * can be overwritten by dynamic reconfigure or by messages received
     * on the "footprint" topic. */
    std::vector<commsgs::geometry_msgs::Point> getRobotFootprint()
    {
        return padded_footprint_;
    }

    /** @brief Return the current unpadded footprint of the robot as a vector of points.
     *
     * This is the raw version of the footprint without padding.
     *
     * The footprint initially comes from the rosparam "footprint" but
     * can be overwritten by dynamic reconfigure or by messages received
     * on the "footprint" topic. */
    std::vector<geometry_msgs::msg::Point> getUnpaddedRobotFootprint()
    {
        return unpadded_footprint_;
    }

protected:
    // options for costmap 2D
    proto::Costmap2DOptions options_;

};

proto::Costmap2DOptions CreateCostmap2DOptions(
    ::autonomy::common::LuaParameterDictionary* const parameter_dictionary);

}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy
