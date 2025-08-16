/*
 * Copyright 2024 The OpenRobotic Beginner Authors
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

#include <string>
#include <vector>
#include <unordered_set>

#include "autonomy//commsgs/geometry_msgs.hpp"
#include "autonomy/map/costmap_2d/costmap_2d.hpp"
#include "autonomy/map/costmap_2d/layered_costmap.hpp"

namespace autonomy {
namespace map {
namespace costmap_2d {

class LayeredCostmap;

/**
 * @class Layer
 * @brief Abstract class for layered costmap plugin implementations
 */
class Layer
{
public:
    /**
     * @brief A constructor
     */
    Layer();

    /**
     * @brief A destructor
     */
    virtual ~Layer() {}

    /**
     * @brief Initialization process of layer on startup
     */
    void initialize(LayeredCostmap* parent, std::string name);

    /**
     * @brief Reset this costmap
     */
    virtual void reset() = 0;

    /**
     * @brief If clearing operations should be processed on this layer or not
     */
    virtual bool isClearable() = 0;

    /**
     * @brief This is called by the LayeredCostmap to poll this plugin as to how
     *        much of the costmap it needs to update. Each layer can increase
     *        the size of this bounds.
     *
     * For more details, see "Layered Costmaps for Context-Sensitive Navigation",
     * by Lu et. Al, IROS 2014.
     */
    virtual void updateBounds(
        double robot_x, double robot_y, double robot_yaw, double * min_x,
        double * min_y,
        double * max_x,
        double * max_y) = 0;

    /**
     * @brief Actually update the underlying costmap, only within the bounds
     *        calculated during UpdateBounds().
     */
    virtual void updateCosts(Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) = 0;

    /** 
     * @brief Implement this to make this layer match the size of the parent costmap. 
     */
    virtual void matchSize() {}

    /** 
     * @brief LayeredCostmap calls this whenever the footprint there
     * changes (via LayeredCostmap::setFootprint()).  Override to be
     * notified of changes to the robot's footprint.
     */
    virtual void onFootprintChanged() {}

    /** 
     * @brief Get the name of the costmap layer
     */
    std::string getName() const
    {
        return name_;
    }

    /**
     * @brief Check to make sure all the data in the layer is up to date.
     *        If the layer is not up to date, then it may be unsafe to
     *        plan using the data from this layer, and the planner may
     *        need to know.
     *
     *        A layer's current state should be managed by the protected
     *        variable current_.
     * @return Whether the data in the layer is up to date.
     */
    bool isCurrent() const
    {
        return current_;
    }

    /**@brief Gets whether the layer is enabled. */
    bool isEnabled() const
    {
        return enabled_;
    }

    /** 
     * @brief  Convenience function for layered_costmap_->getFootprint(). 
     */
    const std::vector<commsgs::geometry_msgs::Point>& getFootprint() const;

    /** @brief Convenience functions for declaring ROS parameters */
    std::string getFullName(const std::string& param_name);

protected:
    LayeredCostmap* layered_costmap_; 
    
    std::string name_;
    
    /** 
     * @brief This is called at the end of initialize().  Override to
     * implement subclass-specific initialization.
     *
     * tf_, name_, and layered_costmap_ will all be set already when this is called.
     */
    virtual void onInitialize() {}

    bool current_;

    // Currently this var is managed by subclasses.
    // TODO(bpwilcox): make this managed by this class and/or container class.
    bool enabled_;

private:
    std::vector<commsgs::geometry_msgs::Point> footprint_spec_;
};

}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy