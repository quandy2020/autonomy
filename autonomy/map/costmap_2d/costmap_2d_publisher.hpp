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


#include <stdexcept>
#include <algorithm>

#include "autonomy/map/costmap_2d/costmap_2d.hpp"

namespace autonomy {
namespace map {
namespace costmap_2d {

/**
 * @class Costmap2DPublisher
 * @brief A tool to periodically publish visualization data from a Costmap2D
 */
class Costmap2DPublisher
{
public:
    /**
     * @brief  Constructor for the Costmap2DPublisher
     */
    Costmap2DPublisher(
        Costmap2D * costmap,
        std::string global_frame,
        std::string topic_name,
        bool always_send_full_costmap = false,
        double map_vis_z = 0.0);

    /**
     * @brief  Destructor
     */
    ~Costmap2DPublisher();

    /** 
     * @brief Include the given bounds in the changed-rectangle. 
     */
    void updateBounds(unsigned int x0, unsigned int xn, unsigned int y0, unsigned int yn)
    {
        x0_ = std::min(x0, x0_);
        xn_ = std::max(xn, xn_);
        y0_ = std::min(y0, y0_);
        yn_ = std::max(yn, yn_);
    }

    /**
     * @brief  Publishes the visualization data over ROS
     */
    void publishCostmap();

    /**
     * @brief Check if the publisher is active
     * @return True if the frequency for the publisher is non-zero, false otherwise
     */
    bool active()
    {
        return active_;
    }

private:
    /** @brief Prepare grid_ message for publication. */
    void prepareGrid();
    void prepareCostmap();

    Costmap2D * costmap_;
    std::string global_frame_;
    std::string topic_name_;
    unsigned int x0_, xn_, y0_, yn_;
    double saved_origin_x_;
    double saved_origin_y_;
    bool active_;
    bool always_send_full_costmap_;
    double map_vis_z_;


    float grid_resolution_;
    unsigned int grid_width_, grid_height_;
    // std::unique_ptr<nav_msgs::msg::OccupancyGrid> grid_;
    // std::unique_ptr<nav2_msgs::msg::Costmap> costmap_raw_;
    // Translate from 0-255 values in costmap to -1 to 100 values in message.
    static char * cost_translation_table_;
};

}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy